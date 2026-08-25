"""
Filename: hermes/revalexo/ai_fatigue/utils/feature_extraction.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-06-25
Version: 1.0
Description: Feature extraction for live fatigue inference.

    Final model vector = 9 ECG features (alphabetically sorted) followed by 30 pelvis
    IMU features (alphabetically sorted) = 39 continuous features
"""

import warnings
import numpy as np
import pandas as pd
from scipy.signal import butter, sosfilt, welch
from scipy.stats import linregress

try:
    import neurokit2 as nk

    _HAS_NK = True
except ImportError:
    _HAS_NK = False


# ============================================================
# Constants
# ============================================================
GLOBAL_WIN_SEC = 60.0  # level-60 window length
SUB_WIN_SEC = 0.5  # IMU sub-window
SUB_STRIDE_SEC = 0.25  # IMU sub-window stride

# ECG / HRV gating
QRS_BAND_HZ = (5.0, 25.0)
ECG_POLARITY_PREFIX_S = 30.0
RR_MIN_S = 0.30
RR_MAX_S = 1.50
MIN_PEAKS_WINDOW = 5
MIN_RR_VALID = 4
DFA_WIN_MIN = 4
DFA_WIN_MAX = 16

# IMU
IMU_LP_HZ = 20.0

# Missing-data / gap policy:
#   - short gaps (<= GAP_MAX_BRIDGE_S) are forward-filled (causal hold of last value)
#   - longer gaps and any LEADING NaN stay NaN (0.0 in the scaler)
#   - a window with finite fraction < ECG_MIN_FINITE_FRAC (ECG) is gated to all-NaN
GAP_MAX_BRIDGE_S = 0.5
ECG_MIN_FINITE_FRAC = 0.5

# Per-modality raw feature names BEFORE the alphabetical sort that data.py applies.
_ECG_RAW_NAMES = [
    "ECG_Mean_HR",
    "ECG_Mean_RR",
    "ECG_SDNN",
    "ECG_RMSSD",
    "ECG_pNN50",
    "ECG_HF_Power",
    "ECG_SD2",
    "ECG_DFA_alpha1",
    "ECG_HR_Slope",
]
# 10 IMU base features.
_IMU_BASES = [
    "vm",
    "sma",
    "acc_rms_x",
    "acc_rms_y",
    "acc_rms_z",
    "gyr_mag",
    "jerk_rms",
    "freq_x",
    "freq_y",
    "freq_z",
]
_IMU_STATS = ["mean", "std", "slope"]

ECG_FEATURE_NAMES = sorted(_ECG_RAW_NAMES)
IMU_FEATURE_NAMES = sorted(
    f"IMU_Pelvis_{base}_{stat}" for base in _IMU_BASES for stat in _IMU_STATS
)


def all_feature_names() -> list[str]:
    """Ordered 39-feature names: ECG block (alpha) then IMU block (alpha).

    This order is the one the saved StandardScaler (mean/scale) and the PF
    fusion mod_slices are built against.
    """
    return ECG_FEATURE_NAMES + IMU_FEATURE_NAMES


def modality_slices() -> dict[str, tuple[int, int]]:
    n_ecg = len(ECG_FEATURE_NAMES)
    n_imu = len(IMU_FEATURE_NAMES)
    return {"ecg": (0, n_ecg), "imu": (n_ecg, n_ecg + n_imu)}


# ============================================================
# Causal preprocessing
# ============================================================


def _causal_butter(data, fs, cutoff, btype, order=4):
    nyq = fs / 2.0
    if btype == "bandpass":
        hi = min(cutoff[1], nyq - 1.0)
        lo = min(cutoff[0], hi - 0.5)
        cutoff = [lo, hi]
    elif btype == "lowpass":
        if cutoff >= nyq:
            cutoff = nyq - 1.0
    elif btype == "highpass":
        if cutoff >= nyq:
            return data
    sos = butter(order, cutoff, btype=btype, fs=fs, output="sos")
    return sosfilt(sos, data)


def _causal_gap_filter(
    x: np.ndarray, fs: float, filt, max_gap_s: float = GAP_MAX_BRIDGE_S
) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    n = len(x)
    if n == 0:
        return x
    nlim = max(1, int(round(max_gap_s * fs)))
    filled = pd.Series(x).ffill(limit=nlim).to_numpy()
    out = np.full(n, np.nan, dtype=np.float64)
    finite = np.isfinite(filled)
    i = 0
    while i < n:
        if not finite[i]:
            i += 1
            continue
        j = i
        while j < n and finite[j]:
            j += 1
        out[i:j] = filt(filled[i:j])
        i = j
    return out


def _interp_within(x: np.ndarray):
    x = np.asarray(x, dtype=np.float64)
    m = np.isfinite(x)
    n_nan = int((~m).sum())
    if n_nan == 0 or m.sum() < 2:
        return x, n_nan
    idx = np.arange(len(x))
    out = x.copy()
    out[~m] = np.interp(idx[~m], idx[m], x[m])
    return out, n_nan


# ============================================================
# ECG / HRV  (9 features)
# ============================================================


def _detect_rpeaks(ecg_raw: np.ndarray, fs: float):
    if not _HAS_NK:
        raise ImportError("neurokit2 is required for ECG feature extraction")
    if ecg_raw is None or len(ecg_raw) < int(0.5 * fs):
        return None

    det = _causal_gap_filter(
        ecg_raw,
        fs,
        lambda s: _causal_butter(s, fs, list(QRS_BAND_HZ), "bandpass"),
    )

    fin = np.isfinite(det)
    if fin.mean() < ECG_MIN_FINITE_FRAC:
        return None

    # Polarity: over the first 30s of finite samples, if trough dominates peak, invert.
    prefix = det[: int(min(len(det), ECG_POLARITY_PREFIX_S * fs))]
    pf = prefix[np.isfinite(prefix)]
    if pf.size and abs(pf.min()) > abs(pf.max()):
        det = -det

    # Fill only a few gap-edge NaNs within this window so the detector can run.
    if not fin.all():
        det, _ = _interp_within(det)
        det = det[np.isfinite(det)]
        if len(det) < int(0.5 * fs):
            return None

    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            _, info = nk.ecg_peaks(
                det,
                sampling_rate=int(round(fs)),
                method="neurokit",
                correct_artifacts=True,
            )
        return np.asarray(info.get("ECG_R_Peaks", []), dtype=int)
    except Exception:
        return np.array([], dtype=int)


def extract_ecg_features(rpeak_idx: np.ndarray, fs: float) -> dict:
    """9 HRV features from a window's R-peaks."""
    out = {k: np.nan for k in _ECG_RAW_NAMES}
    if rpeak_idx is None or len(rpeak_idx) < MIN_PEAKS_WINDOW:
        return out

    rr_s = np.diff(rpeak_idx) / fs
    rr_s = np.where((rr_s >= RR_MIN_S) & (rr_s <= RR_MAX_S), rr_s, np.nan)
    rr_finite = rr_s[np.isfinite(rr_s)]
    if len(rr_finite) < MIN_RR_VALID:
        return out

    rr_ms = rr_s * 1000.0
    rr_ms_finite = rr_finite * 1000.0

    dd = np.diff(rr_ms)
    dd = dd[np.isfinite(dd)]

    out["ECG_Mean_RR"] = float(np.mean(rr_ms_finite))
    out["ECG_Mean_HR"] = 60000.0 / out["ECG_Mean_RR"]
    out["ECG_SDNN"] = (
        float(np.std(rr_ms_finite, ddof=1)) if len(rr_ms_finite) > 1 else 0.0
    )
    out["ECG_RMSSD"] = float(np.sqrt(np.mean(dd**2))) if len(dd) > 0 else 0.0
    out["ECG_pNN50"] = (
        (100.0 * float(np.sum(np.abs(dd) > 50.0)) / len(dd)) if len(dd) > 0 else 0.0
    )
    out["ECG_SD2"] = float(
        np.sqrt(max(0.0, 2.0 * out["ECG_SDNN"] ** 2 - 0.5 * out["ECG_RMSSD"] ** 2))
    )

    try:
        t_cum = np.insert(np.cumsum(rr_finite), 0, 0.0)
        resamp_fs = 4.0
        t_uniform = np.arange(t_cum[0], t_cum[-1], 1.0 / resamp_fs)
        if len(t_uniform) >= 16:
            rri = np.interp(t_uniform, t_cum[1:], rr_finite)
            freqs, psd = welch(rri, fs=resamp_fs, nperseg=min(len(rri), 64))
            hf = (freqs >= 0.15) & (freqs <= 0.40)
            if np.any(hf):
                _trapz = getattr(np, "trapezoid", getattr(np, "trapz"))
                out["ECG_HF_Power"] = float(_trapz(psd[hf], freqs[hf]))
    except Exception:
        pass

    if _HAS_NK and len(rr_ms_finite) >= DFA_WIN_MAX:
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                dfa = nk.fractal_dfa(
                    rr_ms_finite, windows=range(DFA_WIN_MIN, DFA_WIN_MAX + 1)
                )
            out["ECG_DFA_alpha1"] = float(dfa[0] if isinstance(dfa, tuple) else dfa)
        except Exception:
            pass

    try:
        beat_t = rpeak_idx[1:] / fs
        valid = np.isfinite(rr_ms)
        if valid.sum() >= 3:
            hr_inst = 60000.0 / rr_ms[valid]
            out["ECG_HR_Slope"] = float(linregress(beat_t[valid], hr_inst).slope)
    except Exception:
        pass

    return out


# ============================================================
# IMU  (30 pelvis features)
# ============================================================


def preprocess_imu_window(imu_xyz: np.ndarray, fs: float) -> np.ndarray:
    out = np.empty((imu_xyz.shape[0], imu_xyz.shape[1]), dtype=np.float64)
    for i in range(imu_xyz.shape[1]):
        out[:, i] = _causal_gap_filter(
            imu_xyz[:, i],
            fs,
            lambda s: _causal_butter(s, fs, IMU_LP_HZ, "lowpass"),
        )
    return out


def _welch_peak_freq(x: np.ndarray, fs: float) -> float:
    x = np.asarray(x, dtype=np.float64)
    x = x[np.isfinite(x)]
    if len(x) < 8 or fs <= 0:
        return float("nan")
    freqs, psd = welch(x, fs=fs, nperseg=min(len(x), 256))
    if psd.sum() <= 0:
        return float("nan")
    return float(freqs[int(np.argmax(psd))])


def _imu_subwin_base(seg: np.ndarray, fs: float) -> dict:
    """10 base features for one 0.5s sub-window. seg (n, 6)=[ax,ay,az,gx,gy,gz]."""
    ax, ay, az, gx, gy, gz = (seg[:, i] for i in range(6))
    dt = 1.0 / fs
    jx, jy, jz = np.diff(ax) / dt, np.diff(ay) / dt, np.diff(az) / dt
    jmag = np.sqrt(jx**2 + jy**2 + jz**2)
    return {
        "vm": float(np.mean(np.sqrt(ax**2 + ay**2 + az**2))),
        "sma": float(np.mean(np.abs(ax) + np.abs(ay) + np.abs(az))),
        "acc_rms_x": float(np.sqrt(np.mean(ax**2))),
        "acc_rms_y": float(np.sqrt(np.mean(ay**2))),
        "acc_rms_z": float(np.sqrt(np.mean(az**2))),
        "gyr_mag": float(np.mean(np.sqrt(gx**2 + gy**2 + gz**2))),
        "jerk_rms": float(np.sqrt(np.mean(jmag**2))) if jmag.size else 0.0,
        "freq_x": _welch_peak_freq(ax, fs),
        "freq_y": _welch_peak_freq(ay, fs),
        "freq_z": _welch_peak_freq(az, fs),
    }


def _agg_mean_std_slope(values: np.ndarray, centers: np.ndarray):
    v = np.asarray(values, dtype=np.float64)
    finite = np.isfinite(v)
    mean = float(np.nanmean(v)) if finite.any() else float("nan")
    std = float(np.nanstd(v, ddof=1)) if finite.sum() > 1 else 0.0
    if finite.sum() >= 3:
        try:
            slope = float(linregress(centers[finite], v[finite]).slope)
        except Exception:
            slope = float("nan")
    else:
        slope = float("nan")
    return mean, std, slope


def extract_imu_features(imu_filt: np.ndarray, fs: float) -> dict:
    """30 pelvis IMU features: 10 bases x {mean, std, slope} over 0.5s/0.25s sub-windows."""
    out = {n: np.nan for n in IMU_FEATURE_NAMES}
    n_total = len(imu_filt)
    sub_n = max(1, int(round(SUB_WIN_SEC * fs)))
    step = max(1, int(round(SUB_STRIDE_SEC * fs)))
    if n_total < max(2, int(SUB_WIN_SEC * fs * 0.5)):
        return out

    sw = []
    start = 0
    while start + sub_n <= n_total:
        sw.append((start, start + sub_n))
        start += step
    if not sw and n_total >= max(2, int(0.5 * sub_n)):
        sw.append((0, n_total))
    if not sw:
        return out

    centers = np.array([((lo + hi) // 2) / fs for lo, hi in sw], dtype=np.float64)
    bases = [_imu_subwin_base(imu_filt[lo:hi], fs) for lo, hi in sw]
    for base in _IMU_BASES:
        vals = np.array([b[base] for b in bases], dtype=np.float64)
        mean, std, slope = _agg_mean_std_slope(vals, centers)
        out[f"IMU_Pelvis_{base}_mean"] = mean
        out[f"IMU_Pelvis_{base}_std"] = std
        out[f"IMU_Pelvis_{base}_slope"] = slope
    return out


# ============================================================


def extract_window_features(
    ecg_raw: np.ndarray | None,
    imu_raw: np.ndarray | None,
    fs_ecg: float,
    fs_imu: float,
) -> dict:
    feats: dict = {}

    if ecg_raw is not None and len(ecg_raw) > 0 and fs_ecg > 0:
        feats.update(extract_ecg_features(_detect_rpeaks(ecg_raw, fs_ecg), fs_ecg))
    else:
        feats.update({k: np.nan for k in _ECG_RAW_NAMES})

    if imu_raw is not None and len(imu_raw) > 0 and fs_imu > 0:
        feats.update(
            extract_imu_features(preprocess_imu_window(imu_raw, fs_imu), fs_imu)
        )
    else:
        feats.update({n: np.nan for n in IMU_FEATURE_NAMES})

    return feats


def feats_to_vector(feats: dict) -> np.ndarray:
    """Flatten a feature dict to the 39-D float32 vector."""
    return np.array(
        [feats.get(n, np.nan) for n in all_feature_names()], dtype=np.float32
    )
