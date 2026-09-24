"""
Filename: hermes/aidwear/ai_fatigue/utils/handler.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-04-22
Version: 1.0
Description: Subprocess running live fatigue estimation.

    Every inference_period_s (5s) it reads 60s windows from the shared ECG and pelvis
    Occasional true-RPE reports (from the operator via the notes topic) are pulled from
    report_queue and ingested AFTER scoring (causal): they update SysID theta for all
    future windows.
"""

import time
import numpy as np
import torch
from multiprocessing import Queue
from multiprocessing.synchronize import Event as _Event
from queue import Empty

from hermes.utils.time_utils import get_time, init_time
from hermes.aidwear.ai_intent.utils.datastructures import SharedTensorCircularBuffer

from .config import load_config
from .types import Config, FatigueModalityType, FatigueResult, OutputMode
from .feature_extraction import (
    extract_window_features,
    feats_to_vector,
    all_feature_names,
    modality_slices,
)
from .models import get_model_class
from .models.sysid import (
    rebuild as sysid_rebuild,
    SysIdStreamer,
)


class FatigueEstimatorHandler:
    def __init__(
        self,
        ref_time_s: float,
        config_path: str,
        device: str,
        sampling_rates_hz: dict[str, float],
        is_ready_event: _Event,
        is_keep_data_event: _Event,
        is_stop_new_data_event: _Event,
        is_ai_cleanup_event: _Event,
        is_finished_event: _Event,
        input_buffer: dict[FatigueModalityType, dict],
        output_queue: "Queue[FatigueResult]",
        report_queue: "Queue[float]",
    ):
        self._device = torch.device(
            device
            if (device.startswith("cuda") and torch.cuda.is_available())
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
        self._report_queue = report_queue
        self._sampling_rates = sampling_rates_hz

        self._config: Config = load_config(config_path, device=self._device)
        self._scaler_mean = self._config.scaler_mean
        self._scaler_scale = self._config.scaler_scale
        self._clip = self._config.scaler_clip

        # output_mode selects the head that runs. The frozen PF base always runs
        self._mode = self._config.output_mode
        self._pf = self._build_pf(self._config)
        self._sysid = (
            self._build_sysid(self._config) if self._mode == OutputMode.SYSID else None
        )
        self._num_reports = 0
        self._counter = 0
        self._lstm_state = None  # carried per-layer (h, c) LSTM state for streaming
        self._inference_started = False  # True once a full window is first reached

        print(
            f"Fatigue stack ready: device={self._device} | period={self._config.inference_period_s}s | "
            f"features={len(all_feature_names())} | scaler={'on' if self._scaler_mean is not None else 'OFF'} | "
            f"head={self._mode.value}",
            flush=True,
        )

    # ------------------------------------------------------------------ build
    def _build_pf(self, cfg: Config) -> torch.nn.Module:
        cls = get_model_class(cfg.pf.name)
        model = cls(
            modality_slices(),
            proj_dim=cfg.pf.proj_dim,
            arch=cfg.pf.arch,
            head_hidden=cfg.pf.head_hidden,
            fusion_dropout=cfg.pf.fusion_dropout,
            inter_cell_dropout=cfg.pf.inter_cell_dropout,
            head_layernorm=cfg.pf.head_layernorm,
            head_dropout=cfg.pf.head_dropout,
        ).to(self._device)

        if cfg.pf_checkpoint_path:
            bundle = torch.load(
                cfg.pf_checkpoint_path, map_location=self._device, weights_only=False
            )
            sd = (
                bundle.get("state_dict", bundle) if isinstance(bundle, dict) else bundle
            )
            try:
                model.load_state_dict(sd, strict=True)
                print("PF base loaded.", flush=True)
            except RuntimeError:
                missing, unexpected = model.load_state_dict(sd, strict=False)
                print(
                    f"PF base loaded (non-strict): {len(missing)} missing, {len(unexpected)} unexpected.",
                    flush=True,
                )
        else:
            print("No PF checkpoint configured.", flush=True)

        return model.eval()

    def _build_sysid(self, cfg: Config):
        if not cfg.sysid_checkpoint_path:
            print(
                "Warning: no SysID checkpoint — personalization disabled (sysid==base).",
                flush=True,
            )
            return None

        ck = torch.load(
            cfg.sysid_checkpoint_path, map_location=self._device, weights_only=False
        )
        sysid_cfg = ck["cfg"]
        epochs = ck["epochs"]
        ep = cfg.sysid_epoch if cfg.sysid_epoch in epochs else max(epochs)
        model = sysid_rebuild(sysid_cfg, epochs[ep], self._device)
        print(f"SysID loaded: {sysid_cfg} @ epoch {ep}.", flush=True)
        return SysIdStreamer(model, self._device)

    def __call__(self) -> None:
        init_time(ref_time=self._ref_time_s)
        torch.set_grad_enabled(False)
        self._is_ready_event.set()
        self._is_keep_data_event.wait()

        next_inference_s = get_time()
        while not self._is_stop_new_data_event.is_set():
            # Infer every 5s
            while (now := get_time()) < next_inference_s:
                time.sleep(max(0.001, next_inference_s - now))  # not time yet → nap
            self._run_once()
            next_inference_s += self._config.inference_period_s

        self._is_finished_event.set()
        print("Fatigue subprocess finished processing loop.", flush=True)
        self._is_ai_cleanup_event.wait()
        print("Fatigue subprocess exited.", flush=True)

    @staticmethod
    def _measured_fs(n: int, w_start: float, w_end: float, fallback: float) -> float:
        dur = w_end - w_start
        fs = (n - 1) / dur if (w_start > 0.0 and dur > 0 and n > 1) else fallback
        # Considering valid if >0.5x and <2x the fallback
        return fs if 0.5 * fallback <= fs <= 2.0 * fallback else fallback

    @staticmethod
    def _read(buf: SharedTensorCircularBuffer, n: int):
        slices, w_start, w_end = buf.reserve(n)
        arr = (
            torch.cat(slices, dim=0).cpu().numpy()
            if len(slices) > 1
            else slices[0].cpu().numpy()
        )
        buf.release()
        return arr, w_start, w_end

    # TODO: Separate RPE ingestion from the RPE GT collection
    # Depends on if we want as many RPE ingestions as we collect
    # or a systematic one (e.g. 1 per session)
    # Current behavior: ingest each RPE report (once)
    def _drain_reports(self) -> list[float]:
        reports = []
        while True:
            try:
                reports.append(float(self._report_queue.get_nowait()))
            except Empty:
                break
        return reports

    def _run_once(self) -> None:
        start_time_s = get_time()
        window_start_s, window_end_s = [], []
        ecg_arr = imu_arr = None

        nom_fs_ecg = self._sampling_rates["tmsi"]
        nom_fs_imu = self._sampling_rates["imu"]
        fs_ecg, fs_imu = nom_fs_ecg, nom_fs_imu

        ecg_buf = self._input_buffer[FatigueModalityType.TMSI]["ecg"]
        if ecg_buf is not None:
            n = max(
                1,
                int(
                    self._config.modalities[FatigueModalityType.TMSI].receptive_field_s
                    * nom_fs_ecg
                ),
            )
            arr, w0, w1 = self._read(ecg_buf, n)
            ecg_arr = arr.reshape(-1)
            fs_ecg = self._measured_fs(len(ecg_arr), w0, w1, nom_fs_ecg)
            window_start_s.append(w0)
            window_end_s.append(w1)

        imu_buf = self._input_buffer[FatigueModalityType.IMU]["pelvis"]
        if imu_buf is not None:
            n = max(
                1,
                int(
                    self._config.modalities[FatigueModalityType.IMU].receptive_field_s
                    * nom_fs_imu
                ),
            )
            arr, w0, w1 = self._read(imu_buf, n)
            imu_arr = arr
            fs_imu = self._measured_fs(len(imu_arr), w0, w1, nom_fs_imu)
            window_start_s.append(w0)
            window_end_s.append(w1)

        if (ecg_arr is None or len(ecg_arr) == 0) and (
            imu_arr is None or len(imu_arr) == 0
        ):
            return

        # w_start = the window's last sample time (0 = still unfilled).
        # So w_start > 0 for every modality means all windows are full.
        # Based on the receptive field (n already calculated above)
        buffers_filled = all(w > 0.0 for w in window_start_s)
        if not buffers_filled:
            return

        if not self._inference_started:
            self._inference_started = True
            print("Fatigue: buffers filled — starting inference.", flush=True)

        feats = extract_window_features(
            ecg_arr if (ecg_arr is not None and len(ecg_arr) > 0) else None,
            imu_arr if (imu_arr is not None and len(imu_arr) > 0) else None,
            fs_ecg=fs_ecg,
            fs_imu=fs_imu,
        )
        features = feats_to_vector(feats)  # raw 39-D, pre-scaler (for logging)
        vec = features.copy()
        if self._scaler_mean is not None:
            vec = (features - self._scaler_mean) / self._scaler_scale
            vec = np.clip(vec, -self._clip, self._clip)
        vec = np.nan_to_num(vec, nan=0.0).astype(np.float32)

        # Stream one step through the LSTM, carrying its (h, c) state across calls.
        x_t = torch.from_numpy(vec).view(1, 1, -1).to(self._device)  # (1, 1, F)
        hidden, self._lstm_state = self._pf.substrate.encode_step(
            x_t, self._lstm_state
        )  # (1, 1, 32)
        base_seq = self._pf.head(hidden)  # (1, 1)
        h_t = hidden[:, -1, :]  # (1, 32)
        base_t = base_seq[:, -1]  # (1,)
        base_val = float(base_t.item())

        rpe_sysid = (
            self._sysid.step(h_t, base_t) if self._sysid is not None else float("nan")
        )

        # Ingest occasional true-RPE reports
        for r in self._drain_reports():
            if self._sysid is not None:
                self._sysid.ingest_report(r)
            self._num_reports += 1
            print(
                f"[fatigue] ingested RPE report = {r:.1f} (total {self._num_reports})",
                flush=True,
            )

        primary = rpe_sysid if self._mode == OutputMode.SYSID else base_val

        end_time_s = get_time()

        self._output_queue.put(
            FatigueResult(
                rpe=primary,
                rpe_base=base_val,
                rpe_sysid=rpe_sysid,
                features=features.astype(np.float32),
                num_reports=self._num_reports,
                start_time_s=start_time_s,
                end_time_s=end_time_s,
                window_start_s=np.array(window_start_s, dtype=np.float64),
                window_end_s=np.array(window_end_s, dtype=np.float64),
                counter=self._counter,
            )
        )
        self._counter += 1

        print(
            f"[fatigue t={end_time_s:.2f}s | {(end_time_s - start_time_s) * 1000:.0f}ms] "
            f"RPE({self._mode.value})={primary:.2f}  (base={base_val:.2f})",
            flush=True,
        )
