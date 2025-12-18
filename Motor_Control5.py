#!/usr/bin/env python3
import time
from ctypes import *
import ctypes.util

# ---------- Load EPOS library ----------
name = ctypes.util.find_library("EposCmd")
if not name:
    raise OSError("libEposCmd.so not found")
epos = CDLL(name)

# ---------- EPOS prototypes we use ----------
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
epos.VCS_GetEnableState.argtypes = [c_void_p, c_ushort, POINTER(c_bool), POINTER(c_uint)]
epos.VCS_GetPositionIs.argtypes = [c_void_p, c_ushort, POINTER(c_long), POINTER(c_uint)]
epos.VCS_SetPositionProfile.argtypes = [c_void_p, c_ushort, c_uint, c_uint, c_uint, POINTER(c_uint)]
epos.VCS_MoveToPosition.argtypes   = [c_void_p, c_ushort, c_long, c_bool, c_bool, POINTER(c_uint)]
epos.VCS_HaltPositionMovement.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]

# ---------- Small helpers ----------
def get_err_text(code: int) -> str:
    buf = create_string_buffer(1024)
    epos.VCS_GetErrorInfo(code, buf, 1024)
    return buf.value.decode(errors="ignore")

def CHECK(ok, what, perr):
    if not ok:
        raise RuntimeError(f"{what} failed: 0x{perr.value:08X} – {get_err_text(perr.value)}")

def to_signed32(val: int) -> int:
    """Force a value into signed 32-bit range (handle wrap)."""
    val &= 0xFFFFFFFF
    return val - 0x100000000 if val & 0x80000000 else val

# ---------- Connection/config ----------
nodeID   = 1
baudrate = 1000000
timeout  = 500

acceleration   = 4000   # you can tune
deceleration   = 4000
default_speed  = 5000

pErrorCode = c_uint()
keyHandle  = None

# ---------- Basic device helpers ----------
def get_position() -> int:
    p = c_long()
    ec = c_uint()

    ok = epos.VCS_GetPositionIs(keyHandle, nodeID, byref(p), byref(ec))
    CHECK(ok, "GetPositionIs", ec)
    # Ensure signed 32-bit semantics:
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

def clear_fault_and_reenable():
    ec = c_uint()
    epos.VCS_ClearFault(keyHandle, nodeID, byref(ec))
    epos.VCS_SetEnableState(keyHandle, nodeID, byref(ec))
    time.sleep(0.05)

def print_fault_reason(prefix=""):
    code = pErrorCode.value
    txt  = get_err_text(code)
    if prefix:
        print(f"{prefix} fault 0x{code:08X} – {txt}")
    else:
        print(f"fault 0x{code:08X} – {txt}")

# ---------- Guarded move ----------
def move_to_position_with_guard(target_position: int,
                                target_speed=default_speed,
                                accel=acceleration,
                                decel=deceleration,
                                timeout_s=5.0,
                                stall_window_s=0.5,
                                stall_tol_steps=20,
                                print_every_s=0.3):
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
    last_print = t0
    last_pos   = get_position()

    while True:
        if get_fault():
            halt()
            return get_position(), "fault"

        pos = get_position()
        now = time.time()

        if now - last_print >= print_every_s:
            print(f"  pos={pos} → target={target_position}")
            last_print = now

        if abs(pos - target_position) <= stall_tol_steps:
            return pos, "ok"

        if now - last_check >= stall_window_s:
            if abs(pos - last_pos) <= stall_tol_steps:
                halt()
                return pos, "stall"
            last_pos = pos
            last_check = now

        if now - t0 > timeout_s:
            halt()
            return pos, "timeout"

        time.sleep(0.01)

# ---------- Creep scans for auto-cal ----------
def creep_scan(direction,
               step=1000,
               max_steps=2000,
               speed=default_speed,
               accel=acceleration, decel=deceleration,
               timeout_per_step=0.6,
               stall_tol=20):
    assert direction in (+1, -1)
    pos = get_position()
    for i in range(max_steps):
        target = pos + direction * step
        p, why = move_to_position_with_guard(
            target_position=target,
            target_speed=speed, accel=accel, decel=decel,
            timeout_s=timeout_per_step,
            stall_window_s=0.3,
            stall_tol_steps=stall_tol,
            print_every_s=0.15 if i < 6 else 0.5
        )
        if why == "ok":
            pos = p
            continue
        if why == "stall":
            print(f"[CREEP] stall at {p} after {i} steps")
            return p, "stall"
        if why == "fault":
            print_fault_reason(prefix="[CREEP]")
            clear_fault_and_reenable()
            # back off a little to get off the limit
            _p2, _ = move_to_position_with_guard(
                target_position=pos - direction * (step//2),
                target_speed=max(2000, speed//2),
                accel=accel, decel=decel,
                timeout_s=timeout_per_step
            )
            return p, "fault"
        if why == "timeout":
            print(f"[CREEP] timeout near {p}")
            _p2, _ = move_to_position_with_guard(
                target_position=pos - direction * (step//2),
                target_speed=max(2000, speed//2),
                accel=accel, decel=decel,
                timeout_s=timeout_per_step
            )
            return p, "limit"
    print(f"[CREEP] range exhausted at {pos}")
    return pos, "range"

def auto_calibrate_limits(step=1000,
                          speed=default_speed, accel=acceleration, decel=deceleration,
                          settle_backoff=1500, min_span=3000):
    start = get_position()
    print(f"[CAL] start={start}")

    pos_max, why_p = creep_scan(+1, step=step, speed=speed, accel=accel, decel=decel)
    print(f"[CAL] pos_max={pos_max} ({why_p})")
    time.sleep(0.2)

    _p2, _ = move_to_position_with_guard(
        target_position=pos_max - settle_backoff,
        target_speed=max(2000, speed//2),
        accel=accel, decel=decel,
        timeout_s=1.0
    )
    time.sleep(0.2)

    pos_min, why_m = creep_scan(-1, step=step, speed=speed, accel=accel, decel=decel)
    print(f"[CAL] pos_min={pos_min} ({why_m})")

    # Make sure everything is in signed int32 already
    s_min = to_signed32(pos_min)
    s_max = to_signed32(pos_max)

    # If user wants min=0deg and max=90deg, ensure ordering (min <= max)
    if s_min > s_max:
        # swap to enforce s_min < s_max for mapping
        s_min, s_max = s_max, s_min

    span   = s_max - s_min
    center = s_min + span // 2
    print(f"[CAL] span={span} → center={center}")

    if span < min_span:
        print("⚠️  Very small calibrated span – check soft limits (0x607D) or limit switches.")

    return s_min, s_max, center

# ---------- “Safe” move within calibrated band ----------
def safe_move(target_signed_pos: int,
              speed=default_speed, accel=acceleration, decel=deceleration):
    global POS_MIN, POS_MAX
    tgt = max(POS_MIN, min(POS_MAX, int(target_signed_pos)))
    pos, why = move_to_position_with_guard(
        target_position=tgt,
        target_speed=speed, accel=accel, decel=decel
    )
    print(f"[MOVE] target={target_signed_pos} (clamped {tgt}) -> pos={pos} [{why}]")
    return pos, why

# ---------- Main ----------
if __name__ == "__main__":
    try:
        # ---- Open / enable device
        keyHandle = epos.VCS_OpenDevice(b'EPOS4', b'CANopen', b'CAN_mcp251xfd 0', b'CAN0', byref(pErrorCode))
        if not keyHandle:
            raise RuntimeError(f"OpenDevice returned NULL: 0x{pErrorCode.value:08X} – {get_err_text(pErrorCode.value)}")

        CHECK(epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode)),
              "SetProtocolStackSettings", pErrorCode)
        CHECK(epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode)), "ClearFault", pErrorCode)
        CHECK(epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode)),
              "ActivateProfilePositionMode", pErrorCode)
        CHECK(epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode)), "SetEnableState", pErrorCode)

        # ---- Calibrate travel once
        POS_MIN, POS_MAX, POS_CENTER = auto_calibrate_limits(step=1000, speed=5000)
        print(f"[LIMITS] MIN={POS_MIN}, MAX={POS_MAX}, CENTER={POS_CENTER}")

        # Move to center once (optional)
        safe_move(POS_CENTER, speed=2000)

        # ---- Interactive angle control (0.0 .. 90.0)
        print("\nAngle control ready. Enter degrees 0.0 .. 90.0 (e.g., 46.5). Ctrl-C to quit.\n")
        span = POS_MAX - POS_MIN  # signed span

        while True:
            try:
                raw = input("Angle° (0..90): ").strip()
                # allow empty lines
                if not raw:
                    continue
                # basic “quit” aliases if you ever want them:
                if raw.lower() in ("q", "quit", "exit"):
                    break

                # parse float and round to one decimal
                ang = round(float(raw), 1)
                if ang < 0.0:  ang = 0.0
                if ang > 90.0: ang = 90.0

                # map angle → signed position
                # 0° -> POS_MIN, 90° -> POS_MAX
                frac = ang / 90.0
                target_pos = int(round(POS_MIN + frac * span))

                print(f"→ angle {ang:.1f}° maps to pos {target_pos}")
                safe_move(target_pos, speed=12000,accel=30000, decel=deceleration)

            except ValueError:
                print("Please enter a number like 46.5 (0..90).")
            except KeyboardInterrupt:
                print("\n^C received, stopping…")
                break

    finally:
        # ---- Clean down
        try:
            if keyHandle:
                epos.VCS_SetDisableState(keyHandle, nodeID, byref(pErrorCode))
                epos.VCS_CloseDevice(keyHandle, byref(pErrorCode))
        except Exception:
            pass
