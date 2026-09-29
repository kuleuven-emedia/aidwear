"""
Filename: hermes/aidwear/ai_intent/utils/handler.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2025-12-31
Version: 1.0
Description: Subprocess orchestrating asynchronous computation of the
    AI model, decoupled from the inter-Node data exchange of HERMES.
    Exchanges most recent multimodal data through pinned shared memory
    circular buffers with the parent HERMES Node.
    And asynchronously and in-parallel transfers data over DMA to the
    contiguous CPU/GPU pre-allocated memory for maximum bandwidth, safe
    from overwrites of shared memory data during ongoing DMA transfers.
"""

import os
import numpy as np
import torch
from torch import Tensor, cuda
from torch.nn import Module
from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event

from hermes.utils.time_utils import get_time, init_time

from hermes.nicla_sense_me.utils.types import NiclaLocation

from .datastructures import BufferSlice, SharedTensorCircularBuffer
from .types import Config, InferenceResult, ModalityType, ModelType
from .config import get_modality_type, load_config
from .models import get_model_class
from .transforms import build_transforms
from .utils import (
    preprocess_async_imu,
    preprocess_image,
    preprocess_sync_imu,
    preprocess_video,
)


class IntentClassifierHandler:
    def __init__(
        self,
        ref_time_s: float,
        config_path: str,
        device: str,
        module_params: dict,
        imu_type: str,
        feature_mapping: dict[ModalityType, dict[NiclaLocation | str, int]],
        is_ready_event: _Event,
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        is_ai_cleanup_event: _Event,
        is_finished_event: _Event,
        input_buffer: dict[
            ModalityType, dict[NiclaLocation | str, SharedTensorCircularBuffer]
        ],
        output_queue: "Queue[InferenceResult]",
    ):
        self._device = torch.device(
            device
            if torch.cuda.is_available() or not device.startswith("cuda")
            else "cpu"
        )

        self._ref_time_s = ref_time_s
        self._is_ready_event = is_ready_event
        self._is_keep_data_event = is_keep_data_event
        self._is_stop_new_data_event = is_stop_new_data_event
        self._is_ai_cleanup_event = is_ai_cleanup_event
        self._is_finished_event = is_finished_event

        self._input_buffer = input_buffer
        self._output_queue = output_queue
        self._feature_mapping = feature_mapping
        self._imu_type = imu_type
        self._counter = 0

        self._label_mapping: dict[str, int] = module_params["output_classes"]
        self._num_to_label = {v: k for k, v in self._label_mapping.items()}

        # Contiguous Tensor on CPU/GPU for model input staging.
        self._input_features: dict[ModalityType, dict] = {
            ModalityType(k): v for k, v in module_params["input_features"].items()
        }
        self._staging_tensor = {
            model_type: torch.zeros(
                *details["num_features"],
                dtype=getattr(torch, details["dtype"]),
                device=self._device,
            )
            for model_type, details in self._input_features.items()
        }

        # Construct and prepare the inference-only model.
        self._config = load_config(
            config_path,
            label_mapping=self._label_mapping,
            device=self._device,
        )
        self._modality_type = get_modality_type(self._config)
        self._predictions_dtype = getattr(
            torch, f"uint{self._config.predictions_bit_width}"
        )

        # Initialize independent CUDA streams for parallelizing data transfer of modalities
        self._cuda_streams = {
            mod: cuda.Stream(device=self._device)
            for mod in self._config.modalities.keys()
            if self._device.type == "cuda"
        }

        self._model = self.build_model(self._config, self._device)

        # Build per-modality transforms.
        self._mod_transforms = dict(
            map(
                lambda item: (item[0], build_transforms(item[1].eval_transforms)),
                self._config.modalities.items(),
            ),
        )

        print(
            f"PyTorch pipeline ready: {self._modality_type.name} | device={self._device} | horizons={self._config.prediction_horizons}",
            flush=True,
        )

    def __call__(self) -> None:
        """Run the asynchronous PyTorch inference subprocess.

        The subprocess ingests the receptive field worth of the latest available data
        from the pinned shared memory circular buffers. If a GPU is available, transfers
        the data onto it via DMA, overlapping CPU-GPU data transfer with kernels streaming
        for the maximum bandwidth.
        """

        init_time(ref_time=self._ref_time_s)
        # Globally turn off gradient accumulation.
        torch.set_grad_enabled(False)
        self._is_ready_event.set()
        self._is_keep_data_event.wait()

        while not self._is_stop_new_data_event.is_set():
            ###############################################################################
            # TODO: Trigger inference conditionally (e.g. w.r.t the biomechanical context).
            #   Currently runs as fast as the model can, using the latest available data.
            ###############################################################################

            buffer_slices: dict[str, BufferSlice] = {}
            releasers: list = []
            start_time_s: float = get_time()

            if ModalityType.RAW_IMU in self._config.modalities:
                stream = self._cuda_streams.get(ModalityType.RAW_IMU)
                with cuda.stream(stream):
                    if self._imu_type == "mvn":
                        buf_slice, rel = preprocess_sync_imu(
                            staging_tensor=self._staging_tensor[ModalityType.RAW_IMU],
                            imu_buffer=self._input_buffer[ModalityType.RAW_IMU]["mvn"],
                            num_samples=self._config.modalities[
                                ModalityType.RAW_IMU
                            ].receptive_field,
                            transforms=self._mod_transforms[ModalityType.RAW_IMU],
                        )
                    else:
                        buf_slice, rel = preprocess_async_imu(
                            staging_tensor=self._staging_tensor[ModalityType.RAW_IMU],
                            imu_buffers=self._input_buffer[ModalityType.RAW_IMU],
                            num_samples=self._config.modalities[
                                ModalityType.RAW_IMU
                            ].receptive_field,
                            feature_mapping=self._feature_mapping[ModalityType.RAW_IMU],
                            transforms=self._mod_transforms[ModalityType.RAW_IMU],
                        )
                    buffer_slices[ModalityType.RAW_IMU.value] = buf_slice
                    releasers.extend(rel)

            if ModalityType.VIDEO in self._config.modalities:
                stream = self._cuda_streams.get(ModalityType.VIDEO)
                with cuda.stream(stream):
                    buf_slice, rel = preprocess_video(
                        staging_tensor=self._staging_tensor[ModalityType.VIDEO],
                        frame_buffer=self._input_buffer[ModalityType.VIDEO]["ego"],
                        num_frames=self._config.modalities[
                            ModalityType.VIDEO
                        ].receptive_field,
                        stride=self._config.modalities[ModalityType.VIDEO].stride,
                        transforms=self._mod_transforms[ModalityType.VIDEO],
                    )
                    buffer_slices[ModalityType.VIDEO.value] = buf_slice
                    releasers.extend(rel)

            if ModalityType.IMAGE in self._config.modalities:
                stream = self._cuda_streams.get(ModalityType.IMAGE)
                with cuda.stream(stream):
                    buf_slice, rel = preprocess_image(
                        staging_tensor=self._staging_tensor[ModalityType.IMAGE],
                        frame_buffer=self._input_buffer[ModalityType.IMAGE]["ego"],
                        transforms=self._mod_transforms[ModalityType.IMAGE],
                    )
                    buffer_slices[ModalityType.IMAGE.value] = buf_slice
                    releasers.extend(rel)

            # Sync all streams to ensure copies are complete before releasing the shared buffer locks.
            if self._device.type == "cuda":
                cuda.synchronize(self._device)

            # Release all shared memory locks immediately after copies are confirmed done.
            # This minimizes the time the producer is blocked from writing new data.
            for release_fn in releasers:
                release_fn()

            inputs = {mod: buf_slice.tensor for mod, buf_slice in buffer_slices.items()}
            if self._modality_type == ModalityType.MULTIMODAL:
                logits: Tensor = self._model(**inputs)
            else:
                logits: Tensor = self._model(*inputs.values())

            # Collate the timings of encapsulated receptive field windows of the current inference, per-stream.
            window_start_s: np.ndarray = np.concatenate(
                [buf_slice.window_start_s for buf_slice in buffer_slices.values()],
                dtype=np.float64,
            )
            window_end_s: np.ndarray = np.concatenate(
                [buf_slice.window_end_s for buf_slice in buffer_slices.values()],
                dtype=np.float64,
            )

            # Ensure inference is complete before sending results
            if self._device.type == "cuda":
                cuda.synchronize(self._device)

            end_time_s: float = get_time()

            predictions = logits.argmax(dim=-1)

            self._output_queue.put(
                InferenceResult(
                    logits.squeeze(1).cpu().numpy(),
                    predictions.squeeze(1).cpu().to(self._predictions_dtype).numpy(),
                    start_time_s,
                    end_time_s,
                    window_start_s,
                    window_end_s,
                    self._counter,
                )
            )
            self._counter += 1

            # Print the prediction to the CLI.
            parts = [
                f"[t={end_time_s:.2f}s | {(end_time_s - start_time_s) * 1000:.1f}ms]"
            ]
            for i, h in enumerate(self._config.prediction_horizons):
                pred = predictions[i].item()
                label = self._num_to_label[pred]
                parts.append(f"h={h}s: {label}")
            print(parts, flush=True)

        self._is_finished_event.set()
        print("PyTorch subprocess finished processing loop.", flush=True)
        self._is_ai_cleanup_event.wait()
        print("PyTorch subprocess exited.", flush=True)

    def build_model(self, config: Config, device: str | torch.device) -> Module:
        """Build inference-only model from configuration

        Args:
            config (Config): Top-level configuration of the live AI model.
            device (str | torch.device): Device on which to run the AI inference of the built model.

        Returns:
            Module: Model in eval mode, built from the config, and loaded from the checkpoint.
        """

        models: dict[str, Module] = {}
        for model_type, model_config in config.models.items():
            if model_type == ModelType.FUSION:
                continue
            cls = get_model_class(model_config.name)
            params = model_config.params.copy()
            params.setdefault("num_classes", config.num_classes)
            params.setdefault("prediction_horizons", config.prediction_horizons)
            models[model_type.value] = cls(**params).to(device)

        if ModelType.FUSION in config.models:
            fusion_config = config.models[ModelType.FUSION]
            params = fusion_config.params.copy()
            params.setdefault("num_classes", config.num_classes)
            params.setdefault("prediction_horizons", config.prediction_horizons)
            encoders: dict[str, Module] = {}
            for mod in config.modalities.keys():
                if mod.value in models:
                    encoders[mod.value] = models[mod.value]
            params["modality_encoders"] = encoders
            main_model: Module = get_model_class(fusion_config.name)(**params)
            main_model = main_model.to(device)
        else:
            main_model = next(
                models[k]
                for k in [
                    ModelType.VIDEO.value,
                    ModelType.IMAGE.value,
                    ModelType.RAW_IMU.value,
                ]
                if k in models
            )

        # Load checkpoint.
        if config.checkpoint_path and os.path.exists(config.checkpoint_path):
            ckpt: dict = torch.load(
                config.checkpoint_path, map_location=device, weights_only=False
            )
            sd = ckpt.get("model_state_dict", ckpt.get("state_dict", ckpt))
            try:
                main_model.load_state_dict(sd, strict=True)
                print("Checkpoint loaded (strict)", flush=True)
            except RuntimeError:
                missing, unexpected = main_model.load_state_dict(sd, strict=False)
                print(
                    f"Checkpoint loaded (non-strict): {len(missing)} missing, {len(unexpected)} unexpected keys",
                    flush=True,
                )
        elif config.checkpoint_path:
            print(
                f"Warning: Checkpoint not found: {config.checkpoint_path}", flush=True
            )

        # Compile the model with Dynamo for higher efficiency.
        # TODO: compile with TensoRT or ONNX in the future.
        # return torch.compile(main_model.eval(), mode="reduce-overhead")
        return main_model.eval()
