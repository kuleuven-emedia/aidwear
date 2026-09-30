"""
Filename: hermes/aidwear/ai_intent/pipeline.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-16
Version: 1.0
Description: HERMES Node that wraps interaction between the HERMES sensing
    framework and a custom live inference PyTorch AI model.
"""

from multiprocessing import Process, Event, Queue
from queue import Empty
import re
from typing import Optional
import numpy as np
import torch
import torchvision

from hermes.base.nodes.pipeline import Pipeline
from hermes.utils.time_utils import get_time
from hermes.utils.types import LoggingSpec, VideoFormatEnum, NewData
from hermes.utils.mp_utils import launch_handler
from hermes.utils.zmq_utils import (
    PORT_BACKEND,
    PORT_FRONTEND,
    PORT_SYNC_HOST,
    PORT_KILL,
)

from hermes.nicla_sense_me.utils.types import NiclaDataGetMethods, NiclaLocation

from .data_container import IntentClassifierDataContainer
from .utils.datastructures import SharedTensorCircularBuffer
from .utils.handler import IntentClassifierHandler
from .utils.types import InferenceResult, ModalityType


class IntentClassifierPipeline(Pipeline):
    """A HERMES Pipeline node that runs intent-based locomotion mode switching AI model on live multimodal data."""

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
        # Wrap AI compute in a separate process to not stall ZeroMQ.
        self._is_ready_event = Event()
        self._is_keep_data_event = Event()
        self._is_stop_new_data_event = Event()
        self._is_ai_cleanup_event = Event()
        self._is_finished_event = Event()

        # Check if video data is used and if images arrive in MJPEG format.
        self._is_mjpeg = any(
            filter(
                lambda x: (
                    x["settings"].get("video_image_format", None)
                    == VideoFormatEnum.MJPEG.name
                ),
                data_in_specs,
            )
        )

        # Define scaling factors for converting raw Nicla IMU data to real-world units.
        self._imu_type = data_out_spec["imu_type"]
        self._gravity_scaling_factor = (
            data_out_spec["gravity_scaling_factor"] * 9.80665 / 32768.0
        )
        self._gyroscope_scaling_factor = (
            data_out_spec["gyroscope_scaling_factor"] * 180.0 / np.pi / 32768.0
        )

        # Instantiate shared memory torch circular buffers.
        self._torch_device: str = data_out_spec["device"]
        input_modalities_spec: dict[str, dict[str, dict]] = data_out_spec["modalities"]
        self._input_buffer: dict[
            ModalityType, dict[NiclaLocation | str, SharedTensorCircularBuffer]
        ] = {
            ModalityType(modality): {
                NiclaLocation(dev)
                if ModalityType(modality) == ModalityType.RAW_IMU
                and self._imu_type != "mvn"
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
                if ModalityType(modality) == ModalityType.RAW_IMU
                and self._imu_type != "mvn"
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
                "config_path": data_out_spec["config_path"],
                "device": data_out_spec["device"],
                "module_params": data_out_spec["module_params"],
                "imu_type": self._imu_type,
                "feature_mapping": feature_mapping,
                "is_ready_event": self._is_ready_event,
                "is_keep_data_event": self._is_keep_data_event,
                "is_stop_new_data_event": self._is_stop_new_data_event,
                "is_ai_cleanup_event": self._is_ai_cleanup_event,
                "is_finished_event": self._is_finished_event,
                "input_buffer": self._input_buffer,
                "output_queue": self._output_queue,
            },
        )
        self._handler_proc.start()
        self._is_ready_event.wait()

        data_out_spec = {
            "classes": data_out_spec["module_params"]["output_classes"],
            "horizons": data_out_spec["module_params"]["prediction_horizons"],
            "in_streams": [
                key for mod in input_modalities_spec.values() for key in mod
            ],
            "buf_len": data_out_spec["buf_len"],
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
    def create_data_container(cls, data_spec: dict) -> IntentClassifierDataContainer:
        return IntentClassifierDataContainer(**data_spec)

    def _keep_samples(self):
        self._is_keep_data_event.set()

    def _process_data(self, topic: str, msg: NewData) -> None:
        # Places incoming samples into the correct part of the pinned shared circular buffers,
        #   along with timestamps to record metadata of what data was ingested by the online model.
        # TODO: replace hardcoded topic strings with specifications from YAML file
        if topic in ["prosthesis", "niclas"]:
            # Get only the IMU data from the prosthesis packets (arrive in unrolled Numpy arrays of potentially different length).
            #   Not all sensors may have a new value.
            # Nicla IMU feature order matching for the AI model is done in the config file (order of `modalities`), no extra steps needed.
            # Flip Nicla axes orientations to match Awinda from AidWear alpha.
            #   [x,y,z] -> [-y,-x,-z] (torso used in place of pelvis)
            #   [x,y,z] -> [-y,-z,x] (thigh right, shank right)
            #   [x,y,z] -> [-y,z,-x] (thigh left, shank left)
            # Gyroscope from Nicla is in degrees/second, while Xsens-trained AI model is in rad/s; gyro scaling factor autoconverts this.
            
            #   [x,y,z] -> [-y,x,z] (pelvis)
            filtered_niclas = {
                k: v for k, v in msg.items()
                if re.match(
                    f"^nicla_(?!{'|'.join([
                        NiclaLocation.PELVIS.value,
                        NiclaLocation.FOOT_LEFT.value,
                        NiclaLocation.FOOT_RIGHT.value,
                    ])})(.*)$",
                    k
                )
            }

            enum_mapped_niclas = {
                NiclaLocation(k.split("nicla_")[1]): v
                for k, v in filtered_niclas.items()
            }

            data: dict[NiclaLocation, tuple[torch.Tensor, torch.Tensor]] = {
                k: (
                    torch.from_numpy( 
                        np.concatenate(
                            (
                                v[NiclaDataGetMethods.acceleration.name][:, [1, 0, 2]]
                                * self._gravity_scaling_factor
                                * [-1, 1, 1],
                                # v[NiclaDataGetMethods.gyroscope.name][:, [1, 0, 2]]
                                # * self._gyroscope_scaling_factor
                                # * [-1, 1, 1]
                            ),
                            axis=1,
                            dtype=np.float32,
                        )
                    ),
                    torch.from_numpy(v["toa_s"]),
                )
                for k, v in enum_mapped_niclas.items()
            }

            # data: dict[NiclaLocation, tuple[torch.Tensor, torch.Tensor]] = {
            #     k: (
            #         torch.from_numpy( 
            #             np.concatenate(
            #                 (
            #                     v[NiclaDataGetMethods.acceleration.name][:,
            #                         [1, 2, 0]
            #                         if k
            #                         in [
            #                             NiclaLocation.SHANK_LEFT,
            #                             NiclaLocation.SHANK_RIGHT,
            #                             NiclaLocation.THIGH_LEFT,
            #                             NiclaLocation.THIGH_RIGHT,
            #                         ]
            #                         else [1, 0, 2]
            #                     ]
            #                     * self._gravity_scaling_factor
            #                     * ([-1, 1, -1]
            #                     if k
            #                     in [
            #                         NiclaLocation.SHANK_LEFT,
            #                         NiclaLocation.THIGH_LEFT,
            #                     ]
            #                     else (
            #                         [-1, -1, 1]
            #                         if k
            #                         in [
            #                             NiclaLocation.SHANK_RIGHT,
            #                             NiclaLocation.THIGH_RIGHT,
            #                         ]
            #                         else [-1, 1, 1]
            #                     )),
            #                     v[NiclaDataGetMethods.gyroscope.name][:,
            #                         [1, 2, 0]
            #                         if k
            #                         in [
            #                             NiclaLocation.SHANK_LEFT,
            #                             NiclaLocation.SHANK_RIGHT,
            #                             NiclaLocation.THIGH_LEFT,
            #                             NiclaLocation.THIGH_RIGHT,
            #                         ]
            #                         else [1, 0, 2]
            #                     ]
            #                     * self._gyroscope_scaling_factor
            #                     * ([-1, 1, -1]
            #                     if k
            #                     in [
            #                         NiclaLocation.SHANK_LEFT,
            #                         NiclaLocation.THIGH_LEFT,
            #                     ]
            #                     else (
            #                         [-1, -1, 1]
            #                         if k
            #                         in [
            #                             NiclaLocation.SHANK_RIGHT,
            #                             NiclaLocation.THIGH_RIGHT,
            #                         ]
            #                         else [-1, 1, 1]
            #                     )),
            #                 ),
            #                 axis=1,
            #                 dtype=np.float32,
            #             )
            #         ),
            #         torch.from_numpy(v["toa_s"]),
            #     )
            #     for k, v in enum_mapped_niclas.items()
            # }

            for k, v in data.items():
                self._input_buffer[ModalityType.RAW_IMU][k].put(*v)

        elif topic == "mvn":
            # MVN lower-body data arrives always 1 sample at a time.
            #   [pelvis, thigh_right, shank_right, foot_right, thigh_left, shank_left, foot_left]
            data: dict = msg.get("xsens_motion_trackers", {})
            if data:
                self._input_buffer[ModalityType.RAW_IMU]["mvn"].put(
                    # Route data features in the order that the model expects.
                    # Data comes in:
                    #   [pelvis_acc_xyz, thigh_right_acc_xyz, shank_right_acc_xyz, foot_right_acc_xyz, thigh_left_acc_xyz, shank_left_acc_xyz, foot_left_acc_xyz,
                    #       pelvis_gyr_xyz, thigh_right_gyr_xyz, shank_right_gyr_xyz, foot_right_gyr_xyz, thigh_left_gyr_xyz, shank_left_gyr_xyz, foot_left_gyr_xyz]
                    # Model expects:
                    #   [pelvis_acc_xyz, thigh_right_acc_xyz, thigh_left_acc_xyz, shank_right_acc_xyz, shank_left_acc_xyz, foot_right_acc_xyz, foot_left_acc_xyz,
                    #       pelvis_gyr_xyz, thigh_right_gyr_xyz, thigh_left_gyr_xyz, shank_right_gyr_xyz, shank_left_gyr_xyz, foot_right_gyr_xyz, foot_left_gyr_xyz]
                    # NOTE: if orienting IMUs differently than RevalExo alpha (LEDs up, shanks laterally near ankles, mid-thighs laterally), flip IMUs' axes correspondingly.
                    new_data=torch.from_numpy(
                        np.concatenate(
                            (
                                data["acceleration"][[0, 1, 4, 2, 5, 3, 6], :],
                                data["gyroscope"][[0, 1, 4, 2, 5, 3, 6], :],
                            ),
                            axis=0,
                            dtype=np.float32,
                        )
                    ).view(1, -1),
                    toa_s=torch.tensor(data["toa_s"], dtype=torch.float64),
                )

        elif topic == "smartglasses":
            # Get only the ego video (arrives in single frames).
            # NOTE: probably better to apply transforms in-place here before putting into the buffer,
            #   to avoid reapplying the same transforms in the model handler every inference loop, but this is simpler for now.
            data: dict = msg.get("ego", {})
            if data:
                if self._is_mjpeg:
                    byte_tensor = torch.frombuffer(data["frame"], dtype=torch.uint8)
                    frame = torchvision.io.decode_jpeg(byte_tensor)
                else:
                    frame = torch.tensor(data["frame"][0]).permute(2, 0, 1)

                self._input_buffer[self._ego_modality_type]["ego"].put(
                    new_data=frame.unsqueeze(0),
                    toa_s=torch.tensor(data["toa_s"], dtype=torch.float64),
                )

    def _generate_data(self) -> None:
        try:
            # Get data from the AI model without blocking.
            result = self._output_queue.get_nowait()
            process_time_s = get_time()
            data = {
                "intent": {
                    "predictions": result.predictions[None],
                    "logits": result.logits[None],
                    "toa_s": np.array([[result.end_time_s]], dtype=np.float64),
                    "compute_time_s": np.array(
                        [[result.end_time_s - result.start_time_s]], dtype=np.float64
                    ),
                    "window_start_s": result.window_start_s[None],
                    "window_end_s": result.window_end_s[None],
                    "sequence_id": np.array([[result.counter]], dtype=np.uint32),
                }
            }
            self._publish(process_time_s=process_time_s, new_data=data)
        except Empty:
            if self._is_finished_event.is_set():
                self._notify_no_more_data_out()

    def _stop_new_data(self) -> None:
        self._is_stop_new_data_event.set()

    def _cleanup(self) -> None:
        self._is_ai_cleanup_event.set()
        self._handler_proc.join()
        super()._cleanup()
