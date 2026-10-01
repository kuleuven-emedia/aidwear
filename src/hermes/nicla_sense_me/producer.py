"""
Filename: hermes/nicla_sense_me/producer.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-20
Version: 1.0
Description: HERMES Node that wraps Nicla Sense ME IMUs
    interfacing into the HERMES fframework.
"""

from multiprocessing import Process, Queue, Event
from typing import Optional
import numpy as np

from hermes.base.nodes.producer import Producer
from hermes.utils.mp_utils import launch_handler
from hermes.utils.zmq_utils import PORT_BACKEND, PORT_SYNC_HOST, PORT_KILL
from hermes.utils.time_utils import get_time
from hermes.utils.types import LoggingSpec

from hermes.nicla_sense_me.utils.types import (
    NiclaData,
    NiclaPayloadMode,
    CalibrationEvent,
)
from .data_container import NiclaSenseMeDataContainer
from .handler import NiclaSenseMeHandler


class NiclaSenseMeProducer(Producer):
    def __init__(
        self,
        node_id: str,
        host_ip: str,
        niclas: dict,
        logging_spec: LoggingSpec,
        buf_len: Optional[int] = 10000,
        port_pub: Optional[str] = PORT_BACKEND,
        port_sync: Optional[str] = PORT_SYNC_HOST,
        port_killsig: Optional[str] = PORT_KILL,
        transmit_delay_sample_period_s: Optional[float] = float("nan"),
        timesteps_before_solidified: Optional[int] = 0,
        **_,
    ):
        # Keyboard stdin input queue.
        self._input_queue: Queue[tuple[float, str]] = _["input_queue"]

        self._nicla_mapping: dict[str, dict] = niclas["device_mapping"]
        self._nicla_data_queue: Queue[tuple[str, float, bytearray]] = Queue()
        self._nicla_payload_mode = NiclaPayloadMode(
            is_acc=niclas["is_acc"],
            is_gyr=niclas["is_gyr"],
            is_mag=niclas["is_mag"],
            is_euler=niclas["is_euler"],
            is_quat=niclas["is_quat"],
            is_temp=niclas["is_temp"],
            is_baro=niclas["is_baro"],
            is_hum=niclas["is_hum"],
        )

        # Synchronization primitives between background exo handler and foreground HERMES procs.
        self._is_ready_event = Event()
        self._is_keep_data_event = Event()
        self._is_stop_new_data_event = Event()
        self._is_dev_cleanup_event = Event()
        self._is_finished_event = Event()
        self._calibration_event_queue: "Queue[CalibrationEvent]" = Queue()

        hermes_kwargs = {
            "ref_time_s": logging_spec.ref_time_s,
            "is_ready_event": self._is_ready_event,
            "is_keep_data_event": self._is_keep_data_event,
            "is_stop_new_data_event": self._is_stop_new_data_event,
            "is_dev_cleanup_event": self._is_dev_cleanup_event,
            "is_finished_event": self._is_finished_event,
            "input_queue": self._input_queue,
            "calibration_event_queue": self._calibration_event_queue,
        }

        self._handler_proc = Process(
            target=launch_handler,
            args=(NiclaSenseMeHandler,),
            kwargs={
                "niclas": niclas,
                "nicla_data_queue": self._nicla_data_queue,
                **hermes_kwargs,
            },
        )
        self._handler_proc.start()

        data_out_spec = {
            "niclas": niclas,
            "buf_len": buf_len,
        }

        super().__init__(
            node_id=node_id,
            host_ip=host_ip,
            data_out_spec=data_out_spec,
            logging_spec=logging_spec,
            port_pub=port_pub,
            port_sync=port_sync,
            port_killsig=port_killsig,
            transmit_delay_sample_period_s=transmit_delay_sample_period_s,
        )

        # Register hierarchical and grouped topics for flexible subscription
        nicla_names = list(self._nicla_mapping.keys())
        nicla_bundles = [f"nicla_{name}" for name in nicla_names]

        topic_map = {
            # Full telemetry / all Niclas
            "telemetry.all": nicla_bundles,
            "telemetry": nicla_bundles,
            "all": nicla_bundles,
            "data": nicla_bundles,
            # Nicla grouping
            "telemetry.nicla.all": nicla_bundles,
            "telemetry.nicla": nicla_bundles,
            "nicla.all": nicla_bundles,
            "nicla": nicla_bundles,
        }

        for name in nicla_names:
            topic_map[f"telemetry.nicla.{name}"] = [f"nicla_{name}"]
            topic_map[f"nicla.{name}"] = [f"nicla_{name}"]
            topic_map[f"nicla_{name}"] = [f"nicla_{name}"]

        self.register_topic_map(topic_map)

    @classmethod
    def create_data_container(cls, data_spec: dict) -> NiclaSenseMeDataContainer:
        return NiclaSenseMeDataContainer(**data_spec)

    def _ping_device(self) -> None:
        return None

    def _connect(self) -> bool:
        self._is_ready_event.wait()
        return True

    def _keep_samples(self) -> None:
        self._is_keep_data_event.set()

    def _process_data(self) -> None:
        if self._is_continue_capture or not self._nicla_data_queue.empty():
            process_time_s = get_time()
            output = {}
            # Nicla data.
            nicla_data: dict[str, list[NiclaData]] = {
                name: [] for name in self._nicla_mapping.keys()
            }
            nicla_toa: dict[str, list[float]] = {
                name: [] for name in self._nicla_mapping.keys()
            }
            while not self._nicla_data_queue.empty():
                nicla_name, toa_s, nicla_sample = self._nicla_data_queue.get_nowait()
                nicla_toa[nicla_name].append(toa_s)
                nicla_data[nicla_name].append(NiclaData.from_bytes(nicla_sample))
            for nicla_name, data in nicla_data.items():
                if data:
                    output[f"nicla_{nicla_name}"] = {
                        "toa_s": np.array(
                            [nicla_toa[nicla_name]], dtype=np.float64
                        ).transpose((1, 0)),
                        "sequence_id": np.array(
                            [list(map(lambda n: n.sequence_id, data))], dtype=np.uint32
                        ).transpose((1, 0)),
                        "timestamp": np.array(
                            [list(map(lambda n: n.timestamp, data))], dtype=np.uint32
                        ).transpose((1, 0)),
                    }
                    for (
                        data_name,
                        data_getter,
                    ) in self._nicla_payload_mode.get_data_getters().items():
                        output[f"nicla_{nicla_name}"][data_name] = data_getter(data)

            if output:
                self._publish(process_time_s=process_time_s, new_data=output)
        elif not self._calibration_event_queue.empty():
            while not self._calibration_event_queue.empty():
                self._calibration_event_queue.get_nowait()
        else:
            self._send_end_packet()

    def _stop_new_data(self) -> None:
        self._is_stop_new_data_event.set()

    def _cleanup(self) -> None:
        self._is_dev_cleanup_event.set()
        self._is_finished_event.wait()
        super()._cleanup()
