#!/usr/bin/env python3
import time, json, os, re
from ctypes import *
import ctypes.util
import asyncio, struct, signal
import numpy as np
from bleak import BleakScanner, BleakClient

# ===================== BLE FILTER (keep only these two) =====================
ALLOWED_SUFFIXES = {"6845", "e0c6"}
def nicla_suffix(name: str) -> str | None:
    if not name:
        return None
    m = re.search(r"NiclaQuat-(\w{4})$", name)
    return m.group(1) if m else None

# ===================== EPOS / MOTOR SETUP =====================
name = ctypes.util.find_library("EposCmd")
if not name:
    raise OSError("libEposCmd.so not found")
epos = CDLL(name)

# EPOS prototypes
epos.VCS_OpenDevice.restype  = c_void_p
epos.VCS_OpenDevice.argtypes = [c_char_p, c_char_p, c_char_p, c_char_p, POINTER(c_uint)]
epos.VCS_GetErrorInfo.argtypes = [c_uint, c_char_p, c_ushort]
epos.VCS_SetProtocolStackSettings.argtypes = [c_void_p, c_uint, c_uint, POINTER(c_uint)]
epos.VCS_ClearFault.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]
epos.VCS_ActivateProfilePositionMode.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]
epos.VCS_SetEnableState.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]
epos.VCS_SetDisableState.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]
epos.VCS_CloseDevice.argtypes = [c_void_p, POINTER(c_uint)]
epos.VCS_GetFaultState.argtypes = [c_void_p, c_ushort, POINTER(c_bool), POINTER(c_uint)]
epos.VCS_GetPositionIs.argtypes = [c_void_p, c_ushort, POINTER(c_long), POINTER(c_uint)]
epos.VCS_SetPositionProfile.argtypes = [c_void_p, c_ushort, c_uint, c_uint, c_uint, POINTER(c_uint)]
epos.VCS_MoveToPosition.argtypes   = [c_void_p, c_ushort, c_long, c_bool, c_bool, POINTER(c_uint)]
epos.VCS_HaltPositionMovement.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]

# torque/current mode 
epos.VCS_ActivateCurrentMode.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]
epos.VCS_SetCurrentMust.argtypes = [c_void_p, c_ushort, c_short, POINTER(c_uint)]

KT = 0.0276       # Nm/A
I_MAX_mA = 3000   # limite massimo corrente

def set_current_mA(tau_des):
    """Converte il torque desiderato [Nm] in corrente [mA] e invia il comando all'EPOS."""
    ec = c_uint()
    current_mA = (tau_des / KT) * 1000
    current_mA = max(min(current_mA, I_MAX_mA), -I_MAX_mA)
    epos.VCS_SetCurrentMust(keyHandle, nodeID, c_short(int(current_mA)), byref(ec))

def get_err_text(code: int) -> str:
    buf = create_string_buffer(1024)
    epos.VCS_GetErrorInfo(code, buf, 1024)
    return buf.value.decode(errors="ignore")

def CHECK(ok, what, perr):
    if not ok:
        raise RuntimeError(f"{what} failed: 0x{perr.value:08X} – {get_err_text(perr.value)}")

def to_signed32(val: int) -> int:
    val &= 0xFFFFFFFF
    return val - 0x100000000 if val & 0x80000000 else val

# EPOS config
nodeID   = 1
baudrate = 1000000
timeout  = 500
acceleration   = 4000
deceleration   = 4000
default_speed  = 5000
pErrorCode = c_uint()
keyHandle  = None
EPOS_LOCK = asyncio.Lock()

# -------- Software angle band
ANGLE_MIN = 0.0
ANGLE_MAX = 120.0
def clamp_angle(a: float) -> float:
    return max(ANGLE_MIN, min(ANGLE_MAX, a))

# --------- SAVE/LOAD LIMITS ----------
LIMITS_PATH = os.path.join(os.path.dirname(__file__), "epos_limits.json")
def save_limits(pos_min: int, pos_max: int):
    try:
        with open(LIMITS_PATH, "w") as f:
            json.dump({"POS_MIN": int(pos_min), "POS_MAX": int(pos_max)}, f)
        print(f"[LIMITS] Saved to {LIMITS_PATH}")
    except Exception as e:
        print(f"[LIMITS] Save failed: {e}")

def load_limits():
    try:
        if not os.path.exists(LIMITS_PATH):
            return None
        with open(LIMITS_PATH, "r") as f:
            data = json.load(f)
        return int(data["POS_MIN"]), int(data["POS_MAX"])
    except Exception as e:
        print(f"[LIMITS] Load failed: {e}")
        return None

def get_position() -> int:
    p = c_long()
    ec = c_uint()
    ok = epos.VCS_GetPositionIs(keyHandle, nodeID, byref(p), byref(ec))
    CHECK(ok, "GetPositionIs", ec)
    return to_signed32(int(p.value))

def get_fault() -> bool:
    fb = c_bool()
    ec = c_uint()
    ok = epos.VCS_GetFaultState(keyHandle, nodeID, byref(fb), byref(ec))
    CHECK(ok, "GetFaultState", ec)
    return bool(fb.value)

def halt():
    ec = c_uint()
    epos.VCS_HaltPositionMovement(keyHandle, nodeID, byref(ec))

def move_to_position_with_guard(target_position: int, target_speed=default_speed,
                                accel=acceleration, decel=deceleration,
                                timeout_s=0.8, stall_window_s=0.25, stall_tol_steps=30):
    ec = c_uint()
    CHECK(epos.VCS_SetPositionProfile(keyHandle, nodeID,
                                      c_uint(int(target_speed)),
                                      c_uint(int(accel)),
                                      c_uint(int(decel)),
                                      byref(ec)),
          "SetPositionProfile", ec)
    CHECK(epos.VCS_MoveToPosition(keyHandle, nodeID,
                                  c_long(int(target_position)),
                                  True, True, byref(ec)),
          "MoveToPosition", ec)
    t0 = time.time()
    last_check = t0
    last_pos   = get_position()
    while True:
        if get_fault():
            halt()
            return get_position(), "fault"
        pos = get_position()
        if abs(pos - target_position) <= stall_tol_steps:
            return pos, "ok"
        now = time.time()
        if now - last_check >= stall_window_s:
            if abs(pos - last_pos) <= stall_tol_steps:
                halt()
                return pos, "stall"
            last_pos = pos
            last_check = now
        if now - t0 > timeout_s:
            halt()
            return pos, "timeout"
        time.sleep(0.001)

# ===================== ZERO LOGICO =====================
pos_zero = None  # Posizione fisica corrispondente a 0° logici
SPAN_ENCODER = None

def angle_to_position(angle_deg: float) -> int:
    a = clamp_angle(angle_deg)
    frac = a / (ANGLE_MAX - ANGLE_MIN)
    return int(round(pos_zero + frac * SPAN_ENCODER))

def position_to_angle(pos: int) -> float:
    delta = pos - pos_zero
    frac = delta / SPAN_ENCODER
    ang = ANGLE_MIN + frac * (ANGLE_MAX - ANGLE_MIN)
    return clamp_angle(ang)

# ===================== PD CONTROL SAFE =====================
last_ang_cur = 0.0
Kp_deg_to_torque = 0.005
Kd_deg_to_torque = 0.0008

async def apply_pd_control_safe(target_angle, dt, torque_max):
    global last_ang_cur, Kp_deg_to_torque, Kd_deg_to_torque
    pos = get_position()
    ang_cur = position_to_angle(pos)
    err = target_angle - ang_cur
    vel = (ang_cur - last_ang_cur) / dt
    last_ang_cur = ang_cur
    tau = Kp_deg_to_torque * err - Kd_deg_to_torque * vel
    tau = max(-torque_max, min(torque_max, tau))
    if abs(err) < 0.5:
        tau = 0
    set_current_mA(tau)

# ===================== SAFE START + PD SQUARE WAVE =====================
async def safe_start_and_run():
    global last_ang_cur, Kp_deg_to_torque, Kd_deg_to_torque, pos_zero, SPAN_ENCODER

    # --- 1️⃣ Zero logico sulla posizione attuale ---
    pos_zero = get_position()
    SPAN_ENCODER = POS_MAX - POS_MIN
    print(f"[SAFE START] Impostato zero logico a posizione fisica {pos_zero}")

    # --- 2️⃣ Attiva modalità corrente/torque ---
    ec = c_uint()
    CHECK(epos.VCS_ActivateCurrentMode(keyHandle, nodeID, byref(ec)),
          "ActivateCurrentMode", ec)

    # --- 3️⃣ Inizializza PD ---
    last_ang_cur = position_to_angle(get_position())

    # --- 4️⃣ Loop combinazioni Kp/Kd + onda quadra ---
    Kp_values = [0.005 + i*0.001 for i in range(6)]
    Kd_values = [0.0008 + j*0.0001 for j in range(9)]

    dt = 0.05
    ANGLE_LOW = 0.0
    ANGLE_HIGH = 70.0
    T_LOW = 2
    T_HIGH = 1
    TORQUE_MAX_SAFE = 0.5

    for kp in Kp_values:
        for kd in Kd_values:
            Kp_deg_to_torque = kp
            Kd_deg_to_torque = kd

            # fase bassa
            t0 = time.time()
            while time.time() - t0 < T_LOW:
                await apply_pd_control_safe(ANGLE_LOW, dt, TORQUE_MAX_SAFE)
                await asyncio.sleep(dt)

            # fase alta
            t1 = time.time()
            ang_raggiunto = None
            while time.time() - t1 < T_HIGH:
                await apply_pd_control_safe(ANGLE_HIGH, dt, TORQUE_MAX_SAFE)
                ang_raggiunto = position_to_angle(get_position())
                await asyncio.sleep(dt)

            # Stampa una volta per fronte alto
            print(f"[ONDA] Kp={kp:.4f}, Kd={kd:.4f}, angolo raggiunto={ang_raggiunto:.2f}")

# ===================== EPOS WATCHDOG =====================
async def epos_watchdog():
    while True:
        try:
            if keyHandle and get_fault():
                print("[WATCHDOG] EPOS FAULT → clearing & re-arming…")
                async with EPOS_LOCK:
                    try:
                        halt()
                    except Exception:
                        pass
        except Exception:
            pass
        await asyncio.sleep(0.005)

# ===================== MAIN =====================
async def main():
    global keyHandle, POS_MIN, POS_MAX

    print("Starting (Nicla→Angle→Motor)…")

    keyHandle = epos.VCS_OpenDevice(b'EPOS4', b'CANopen', b'CAN_mcp251xfd 0', b'CAN0', byref(pErrorCode))
    if not keyHandle:
        raise RuntimeError(f"OpenDevice NULL: 0x{pErrorCode.value:08X} – {get_err_text(pErrorCode.value)}")
    CHECK(epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode)),
          "SetProtocolStackSettings", pErrorCode)
    CHECK(epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode)), "ClearFault", pErrorCode)
    CHECK(epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode)),
          "ActivateProfilePositionMode", pErrorCode)
    CHECK(epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode)), "SetEnableState", pErrorCode)

    POS_MIN = -204011
    POS_MAX = 150464
    print(f"[LIMITS] Using fixed values: MIN={POS_MIN}, MAX={POS_MAX}")
    save_limits(POS_MIN, POS_MAX)

    # Switch a torque mode subito
    CHECK(epos.VCS_ActivateCurrentMode(keyHandle, nodeID, byref(pErrorCode)),
          "ActivateCurrentMode", pErrorCode)

    print("BLE disattivato — controllo motore solo da onda quadra.")

    wd_task  = asyncio.create_task(epos_watchdog())
    ctl_task = asyncio.create_task(safe_start_and_run())

    loop = asyncio.get_event_loop()
    stop_future = loop.create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_future.set_result, None)
    await stop_future

    ctl_task.cancel()
    wd_task.cancel()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        try:
            if keyHandle:
                epos.VCS_SetDisableState(keyHandle, nodeID, byref(pErrorCode))
                epos.VCS_CloseDevice(keyHandle, byref(pErrorCode))
        except Exception:
            pass
