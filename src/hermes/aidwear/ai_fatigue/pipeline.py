"""
Filename: hermes/aidwear/ai_fatigue/pipeline.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-06-25
Version: 1.0
Description: HERMES Node wrapping the live PyTorch fatigue estimation model.
"""

from multiprocessing import Process, Event, Queue
from queue import Empty
from typing import Optional
import numpy as np

from hermes.base.nodes.pipeline import Pipeline
from hermes.utils.time_utils import get_time
from hermes.utils.types import LoggingSpec, NewData
from hermes.utils.mp_utils import launch_handler
from hermes.utils.zmq_utils import (
    PORT_BACKEND,
    PORT_FRONTEND,
    PORT_SYNC_HOST,
    PORT_KILL,
)
import torch

from ..ai_intent.utils.datastructures import SharedTensorCircularBuffer

from .data_container import FatigueEstimatorDataContainer
from .utils.handler import FatigueEstimatorHandler
from .utils.types import FatigueModalityType, FatigueResult

# Tracker order in the Xsens MVN lower-body message; pelvis is index 0.
_MVN_PELVIS_IDX = 0


class FatigueEstimatorPipeline(Pipeline):
    """A HERMES Pipeline node that runs fatigue-based assistance scaling AI model on live multimodal data."""

    def __init__(
        self,
        node_id: str,
        host_ip: str,
        data_out_spec: dict,
        data_in_specs: list[dict],
        logging_spec: LoggingSpec,
        port_pub: Optional[str] = PORT_BACKEND,
        port_sub: Optional[str] = PORT_FRONTEND,
        port_sync: Optional[str] = PORT_SYNC_HOST,
        port_killsig: Optional[str] = PORT_KILL,
        **_,
    ):
        self._is_ready_event = Event()
        self._is_keep_data_event = Event()
        self._is_stop_new_data_event = Event()
        self._is_ai_cleanup_event = Event()
        self._is_finished_event = Event()

        self._torch_device: str = data_out_spec["device"]

        input_modalities_spec: dict[str, dict[str, dict]] = data_out_spec["modalities"]
        self._input_buffer: dict[
            FatigueModalityType, dict[str, SharedTensorCircularBuffer]
        ] = {
            FatigueModalityType(modality_name): {
                dev: SharedTensorCircularBuffer(
                    buf_len=params["buf_len"],
                    num_features=params["num_features"],
                    dtype_str=params["dtype"],
                    device=self._torch_device,
                )
                for dev, params in modality_details.items()
            }
            for modality_name, modality_details in input_modalities_spec.items()
        }

        self._output_queue: Queue[FatigueResult] = Queue()
        self._report_queue: Queue[float] = Queue()  # occasional true-RPE reports

        self._handler_proc = Process(
            target=launch_handler,
            args=(FatigueEstimatorHandler,),
            kwargs={
                "ref_time_s": logging_spec.ref_time_s,
                "config_path": data_out_spec["config_path"],
                "device": data_out_spec["device"],
                "sampling_rates_hz": data_out_spec["sampling_rate_hz"],
                "is_ready_event": self._is_ready_event,
                "is_keep_data_event": self._is_keep_data_event,
                "is_stop_new_data_event": self._is_stop_new_data_event,
                "is_ai_cleanup_event": self._is_ai_cleanup_event,
                "is_finished_event": self._is_finished_event,
                "input_buffer": self._input_buffer,
                "output_queue": self._output_queue,
                "report_queue": self._report_queue,
            },
        )
        self._handler_proc.start()
        self._is_ready_event.wait()

        data_out_spec = {
            "buf_len": data_out_spec["buf_len"],
            "modalities": [
                key for mod in input_modalities_spec.values() for key in mod
            ],
        }

        super().__init__(
            node_id=node_id,
            host_ip=host_ip,
            data_out_spec=data_out_spec,
            data_in_specs=data_in_specs,
            logging_spec=logging_spec,
            is_async_generate=True,
            port_pub=port_pub,
            port_sub=port_sub,
            port_sync=port_sync,
            port_killsig=port_killsig,
        )

    @classmethod
    def create_data_container(cls, data_spec: dict) -> FatigueEstimatorDataContainer:
        return FatigueEstimatorDataContainer(**data_spec)

    def _keep_samples(self):
        self._is_keep_data_event.set()

    def _process_data(self, topic: str, msg: NewData) -> None:
        if topic == "tmsi" and (data := msg.get("tmsi_data", None)):
            self._input_buffer[FatigueModalityType.TMSI]["ecg"].put(
                new_data=torch.from_numpy(data["ecg"]),
                toa_s=torch.from_numpy(data["toa_s"]),
            )

        elif topic == "mvn" and (data := msg.get("xsens_motion_trackers", None)):
            self._input_buffer[FatigueModalityType.IMU]["pelvis"].put(
                new_data=torch.from_numpy(
                    np.concatenate(
                        (
                            data["acceleration"][0, _MVN_PELVIS_IDX, :],
                            data["gyroscope"][0, _MVN_PELVIS_IDX, :],
                        ),
                        axis=0,
                        dtype=np.float32,
                    )
                ).view(1, -1),
                toa_s=torch.from_numpy(data["toa_s"]),
            )

        elif topic == "notes" and (data := msg.get("event", None)):
            self._report_queue.put(float(data["type"].item()))

    def _generate_data(self) -> None:
        try:
            result = self._output_queue.get_nowait()
            process_time_s = get_time()
            data = {
                "fatigue": {
                    "rpe": np.array([[result.rpe]], dtype=np.float32),
                    "rpe_base": np.array([[result.rpe_base]], dtype=np.float32),
                    "rpe_sysid": np.array([[result.rpe_sysid]], dtype=np.float32),
                    "features": result.features[None],
                    "num_reports": np.array([[result.num_reports]], dtype=np.uint32),
                    "toa_s": np.array([[result.end_time_s]], dtype=np.float64),
                    "compute_time_s": np.array(
                        [[result.end_time_s - result.start_time_s]], dtype=np.float64
                    ),
                    "window_start_s": result.window_start_s[None],
                    "window_end_s": result.window_end_s[None],
                    "sequence_id": np.array([[result.counter]], dtype=np.uint32),
                }
            }
            self._publish(process_time_s=process_time_s, new_data=NewData(**data))
        except Empty:
            if self._is_finished_event.is_set():
                self._notify_no_more_data_out()

    def _stop_new_data(self) -> None:
        self._is_stop_new_data_event.set()

    def _cleanup(self) -> None:
        self._is_ai_cleanup_event.set()
        self._handler_proc.join()
        super()._cleanup()
