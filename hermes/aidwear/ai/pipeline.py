"""
Filename: hermes/revalexo/ai/pipeline.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-16
Version: 1.0
Description: HERMES Node that wraps interaction between the HERMES sensing
    framework and a custom live inference PyTorch AI model.
"""

from multiprocessing import Process, Event, Queue
from queue import Empty
import re
import numpy as np
import torch
import torchvision

from hermes.base.nodes.pipeline import Pipeline
from hermes.utils.time_utils import get_time
from hermes.utils.types import LoggingSpec
from hermes.utils.mp_utils import launch_handler
from hermes.utils.zmq_utils import (
    PORT_BACKEND,
    PORT_FRONTEND,
    PORT_SYNC_HOST,
    PORT_KILL,
)

from hermes.aidwear.utils.types import NiclaDataGetMethods, NiclaLocation

from .stream import IntentClassifierStream
from .utils.datastructures import SharedTensorCircularBuffer
from .utils.handler import IntentClassifierHandler
from .utils.types import InferenceResult, ModalityType


class IntentClassifierPipeline(Pipeline):
    """A class for processing realtime sensor data with a custom PyTorch AI model."""

    def __init__(
        self,
        topic: str,
        host_ip: str,
        stream_out_spec: dict,
        stream_in_specs: list[dict],
        logging_spec: LoggingSpec,
        port_pub: str = PORT_BACKEND,
        port_sub: str = PORT_FRONTEND,
        port_sync: str = PORT_SYNC_HOST,
        port_killsig: str = PORT_KILL,
        **_,
    ):
        # Wrap AI compute in a separate process to not stall ZeroMQ.
        self._is_ready_event = Event()
        self._is_keep_data_event = Event()
        self._is_stop_new_data_event = Event()
        self._is_cleanup_event = Event()
        self._is_finished_event = Event()

        # Define scaling factors for converting raw Nicla IMU data to real-world units.
        self._imu_type = stream_out_spec["imu_type"]
        self._gravity_scaling_factor = stream_out_spec["gravity_scaling_factor"] * 9.80665 / 32768.0
        self._gyroscope_scaling_factor = stream_out_spec["gyroscope_scaling_factor"] / 32768.0

        # Instantiate shared memory torch circular buffers.
        self._torch_device: str = stream_out_spec["device"]
        input_modalities_spec: dict[str, dict[str, dict]] = stream_out_spec[
            "modalities"
        ]
        self._input_buffer: dict[
            ModalityType, dict[NiclaLocation | str, SharedTensorCircularBuffer]
        ] = {
            ModalityType(modality): {
                NiclaLocation(dev)
                if ModalityType(modality) == ModalityType.RAW_IMU and self._imu_type != "mvn"
                else dev: SharedTensorCircularBuffer(
                    buf_len=params["buf_len"],
                    num_features=params["num_features"],
                    dtype_str=params["dtype"],
                    device=self._torch_device,
                )
                for dev, params in details.items()
            }
            for modality, details in input_modalities_spec.items()
        }
        self._output_queue: Queue[InferenceResult] = Queue()
        self._ego_modality_type = ModalityType(
            next(
                filter(
                    lambda x: ModalityType(x) != ModalityType.RAW_IMU,
                    input_modalities_spec.keys(),
                ),
                ModalityType.UNKNOWN.value,
            )
        )

        feature_mapping = {
            ModalityType(modality): {
                NiclaLocation(dev)
                if ModalityType(modality) == ModalityType.RAW_IMU and self._imu_type != "mvn"
                else dev: id
                for id, dev in enumerate(details.keys())
            }
            for modality, details in input_modalities_spec.items()
        }

        self._handler_proc = Process(
            target=launch_handler,
            args=(IntentClassifierHandler,),
            kwargs={
                "ref_time_s": logging_spec.ref_time_s,
                "config_path": stream_out_spec["config_path"],
                "device": stream_out_spec["device"],
                "module_params": stream_out_spec["module_params"],
                "imu_type": self._imu_type,
                "feature_mapping": feature_mapping,
                "is_ready_event": self._is_ready_event,
                "is_keep_data_event": self._is_keep_data_event,
                "is_stop_new_data_event": self._is_stop_new_data_event,
                "is_cleanup_event": self._is_cleanup_event,
                "is_finished_event": self._is_finished_event,
                "input_buffer": self._input_buffer,
                "output_queue": self._output_queue,
            },
        )
        self._handler_proc.start()
        self._is_ready_event.wait()

        stream_out_spec = {
            "classes": stream_out_spec["module_params"]["output_classes"],
            "horizons": stream_out_spec["module_params"]["prediction_horizons"],
            "in_streams": [
                key for mod in input_modalities_spec.values() for key in mod
            ],
        }

        super().__init__(
            topic=topic,
            host_ip=host_ip,
            stream_out_spec=stream_out_spec,
            stream_in_specs=stream_in_specs,
            logging_spec=logging_spec,
            is_async_generate=True,
            port_pub=port_pub,
            port_sub=port_sub,
            port_sync=port_sync,
            port_killsig=port_killsig,
        )

    @classmethod
    def create_stream(cls, stream_spec: dict) -> IntentClassifierStream:
        return IntentClassifierStream(**stream_spec)

    def _keep_samples(self):
        self._is_keep_data_event.set()

    def _process_data(self, topic: str, msg: dict) -> None:
        # Places incoming samples into the correct part of the pinned shared circular buffers,
        #   along with timestamps to record metadata of what data was ingested by the online model.
        # TODO: replace hardcoded topic strings with specifications from YAML file
        if topic in ["prosthesis", "wearable_nicla"]:
            # Get only the IMU data from the Exo packets (arrive in unrolled Numpy arrays of potentially different length).
            #   Not all sensors may have a new value.
            data: dict[NiclaLocation, tuple[torch.Tensor, torch.Tensor]] = dict(
                map(
                    lambda items: (
                        NiclaLocation(items[0].split("nicla_")[1]),
                        (
                            torch.from_numpy(
                                np.concatenate(
                                    (
                                        items[1][NiclaDataGetMethods.acceleration.name] * self._gravity_scaling_factor,
                                        items[1][NiclaDataGetMethods.gyroscope.name] * self._gyroscope_scaling_factor,
                                    ),
                                    axis=1,
                                    dtype=np.float32,
                                )
                            ),
                            torch.from_numpy(items[1]["toa_s"]),
                        ),
                    ),
                    filter(
                        lambda items: re.match(
                            f"^nicla_(?!{'|'.join((NiclaLocation.TORSO.value,))})(.*)$", items[0]
                        ),
                        msg["data"].items(),
                    ),
                )
            )

            for k, v in data.items():
                self._input_buffer[ModalityType.RAW_IMU][k].put(*v)

        elif topic == "mvn":
            # MVN lower-body order of features maps one-to-one to AI model inputs. Always 1 sample at a time.
            #   [pelvis, thigh_right, shank_right, foot_right, thigh_left, shank_left, foot_left]
            data: dict = msg["data"].get("xsens-motion-trackers", {})
            if data:
                self._input_buffer[ModalityType.RAW_IMU]["mvn"].put(
                    # Route channels in the correct order for the model.
                    #   [pelvis_acc_xyz, thigh_right_acc_xyz, shank_right_acc_xyz, foot_right_acc_xyz, thigh_left_acc_xyz, shank_left_acc_xyz, foot_left_acc_xyz,
                    #       pelvis_gyr_xyz, thigh_right_gyr_xyz, shank_right_gyr_xyz, foot_right_gyr_xyz, thigh_left_gyr_xyz, shank_left_gyr_xyz, foot_left_gyr_xyz]
                    new_data=torch.from_numpy(
                        np.concatenate(
                            (
                                data["acceleration"],
                                data["gyroscope"],
                            ),
                            axis=0,
                            dtype=np.float32,
                        )
                    ).view(1, -1),
                    toa_s=torch.tensor([[data["toa_s"]]], dtype=torch.float64),
                )

        elif topic == "smartglasses":
            # Get only the ego video (arrives in single frames).
            # TODO: probably better to apply transforms in-place here before putting into the buffer,
            #   to avoid reapplying the same transforms in the model handler every inference loop, but this is simpler for now.
            data: dict = msg["data"].get("ego", {})
            if data:
                raw_frame_data = data["frame"][0]
                if isinstance(raw_frame_data, bytes):
                    byte_tensor = torch.frombuffer(raw_frame_data, dtype=torch.uint8)
                    frame = torchvision.io.decode_jpeg(byte_tensor)
                else:
                    frame = torch.tensor(raw_frame_data)
                self._input_buffer[self._ego_modality_type]["ego"].put(
                    new_data=frame.unsqueeze(0), 
                    toa_s=torch.tensor([[data["toa_s"]]], dtype=torch.float64),
                )

    def _generate_data(self) -> None:
        try:
            # Get data from the AI model without blocking.
            result = self._output_queue.get_nowait()
            process_time_s = get_time()
            data = {
                "predictions": result.predictions,
                "logits": result.logits,
                "toa_s": result.end_time_s,
                "compute_time_s": result.end_time_s - result.start_time_s,
                "window_start_s": result.window_start_s,
                "window_end_s": result.window_end_s,
                "sequence_id": result.counter,
            }
            tag: str = "%s.data" % self.topic
            self._publish(tag, process_time_s=process_time_s, data={"classifier": data})
        except Empty:
            if self._is_finished_event.is_set():
                self._notify_no_more_data_out()

    def _stop_new_data(self) -> None:
        self._is_stop_new_data_event.set()

    def _cleanup(self) -> None:
        self._is_cleanup_event.set()
        self._handler_proc.join()
        super()._cleanup()
