#!/usr/bin/env python3
import time, json, os, re
from ctypes import *
import ctypes.util
import asyncio, struct, signal
import numpy as np
from bleak import BleakScanner, BleakClient

# ===================== BLE FILTER (keep only these two) =====================
ALLOWED_SUFFIXES = {"7616", "6845"}
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



#Velocity mode 
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

# -------- Software angle band (change if you want)
ANGLE_MIN = 2.5
ANGLE_MAX = 87.5
def clamp_angle(a: float) -> float:
    if a < ANGLE_MIN: return ANGLE_MIN
    if a > ANGLE_MAX: return ANGLE_MAX
    return a

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

def clear_fault_and_reenable():
    ec = c_uint()
    epos.VCS_ClearFault(keyHandle, nodeID, byref(ec))
    epos.VCS_SetEnableState(keyHandle, nodeID, byref(ec))
    epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(ec))
    epos.VCS_SetPositionProfile(keyHandle, nodeID,
                                c_uint(int(default_speed)),
                                c_uint(int(acceleration)),
                                c_uint(int(deceleration)),
                                byref(ec))
    time.sleep(0.005)

def move_to_position_with_guard(target_position: int,
                                target_speed=default_speed,
                                accel=acceleration,
                                decel=deceleration,
                                timeout_s=0.8,
                                stall_window_s=0.25,
                                stall_tol_steps=30):
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

def creep_scan(direction,
               step=1000, max_steps=2000,
               speed=default_speed,
               accel=acceleration, decel=deceleration,
               timeout_per_step=0.4, stall_tol=20):
    assert direction in (+1, -1)
    pos = get_position()
    for _ in range(max_steps):
        target = pos + direction * step
        p, why = move_to_position_with_guard(
            target_position=target,
            target_speed=speed, accel=accel, decel=decel,
            timeout_s=timeout_per_step,
            stall_window_s=0.2,
            stall_tol_steps=stall_tol
        )
        if why == "ok":
            pos = p
            continue
        return p, why
    return pos, "range"

# ===== NEW: MAX-ONLY CALIBRATION =====
# If you know the exact mechanical step-span for your usable range, set it here:
DESIRED_SPAN_STEPS = 265000   # e.g. 303047  (set to an int to force)
DEFAULT_SPAN_STEPS = 265000 # fallback if nothing saved and DESIRED_SPAN_STEPS is None


def auto_calibrate_max_only(step=1000,
                            speed=default_speed, accel=acceleration, decel=deceleration,
                            settle_backoff=1500,
                            desired_span=None):
    """
    Find MAX by creeping +1 until stall/timeout, back off a little,
    then define MIN = MAX - desired_span.
    """
    cur = get_position()
    print(f"[CAL] (MAX-only) start={cur}")

    pos_max, why_p = creep_scan(+1, step=step, speed=speed, accel=accel, decel=decel)
    print(f"[CAL] pos_max={pos_max} ({why_p})")

    # Light settle (optional)
    _p2, _ = move_to_position_with_guard(
        target_position=pos_max - settle_backoff,
        target_speed=max(2000, speed//2),
        accel=accel, decel=decel,
        timeout_s=1.0
    )
    time.sleep(0.2)

    # Decide span
    span = int(desired_span)
    if span <= 0:
        raise ValueError("desired_span must be positive")

    s_max = to_signed32(pos_max)
    s_min = s_max - span
    center = s_min + s_max 
    print(f"[CAL] span={span} → MIN={s_min}, MAX={s_max}, center={center}")
    return s_min, s_max, center

# Global limits
POS_MIN = None
POS_MAX = None

def angle_to_position(angle_deg: float) -> int:
    a = clamp_angle(float(angle_deg))
    frac = (a - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN)
    span = POS_MAX - POS_MIN
    return int(round(POS_MIN + frac * span))

def position_to_angle(pos: int) -> float:
    span = float(POS_MAX - POS_MIN)
    if span <= 0:
        return ANGLE_MIN
    frac = (float(pos) - float(POS_MIN)) / span
    ang  = ANGLE_MIN + frac * (ANGLE_MAX - ANGLE_MIN)
    return clamp_angle(ang)

def safe_move(target_signed_pos: int, speed=3000, accel=3000, decel=3000):
    tgt = max(POS_MIN, min(POS_MAX, int(target_signed_pos)))
    pos, why = move_to_position_with_guard(
        target_position=tgt,
        target_speed=speed, accel=accel, decel=decel
    )
    if why in ("fault", "timeout", "stall"):
        try:
            halt()
            clear_fault_and_reenable()
        except Exception:
            pass
        try:
            if POS_MIN is not None and POS_MAX is not None:
                mid = (POS_MIN + POS_MAX) // 2
                back = 200 if tgt >= mid else -200
                _pos, _ = move_to_position_with_guard(
                    target_position=max(POS_MIN, min(POS_MAX, tgt - back)),
                    target_speed=max(1500, speed // 4),
                    accel=max(1000,  accel // 4),
                    decel=max(1000,  decel // 4),
                    timeout_s=1.0
                )
        except Exception:
            pass
        pos, why = move_to_position_with_guard(
            target_position=tgt,
            target_speed=max(1500, speed // 3),
            accel=max(1000,  accel // 3),
            decel=max(1000,  decel // 3),
            timeout_s=1.2
        )


        
    return pos, why

# ===================== BLE / ANGLE SOURCING (pitch diff) =====================
SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214".lower()
CHAR_UUID    = "19b10002-e8f2-537e-4f6c-d104768a1214".lower()

clients = {}

latest_euler_by_suffix = {}  
latest_euler = {}
rx_counts = {}
last_angle_cmd = 0.0
last_cmd_time  = 0.0
latest_target_angle = None
latest_target_time  = 0.0

def make_handler(name, addr):
    def handler(_, data: bytearray):
        nonlocal addr, name
        if len(data) != 28:
            return
        try:
            # ts, gx, gy, gz, pitch, roll, heading
            ts, gx, gy, gz, pitch, roll, heading = struct.unpack("<I6f", data)
        except struct.error:
            return

        # keep your existing per-address store (optional, if you still use it)
        latest_euler[addr] = (heading, pitch, roll)
        rx_counts[addr] = rx_counts.get(addr, 0) + 1

        # NEW: also store by suffix so we can force ordering: 7616 first, 6845 second
        suf = nicla_suffix(name) or ""
        if suf:
            latest_euler_by_suffix[suf] = (heading, pitch, roll)

        # Compute only when we have BOTH
        if "7616" in latest_euler_by_suffix and "6845" in latest_euler_by_suffix:
            e_7616 = latest_euler_by_suffix["7616"]
            e_6845 = latest_euler_by_suffix["6845"]

            # pitch is index 1
            ang = float(e_7616[1] - e_6845[1])   # <-- ORDER IS FIXED HERE

            # clamp to 0..90° then to your allowed band
            ang = max(0.0, min(90.0, ang))
            ang = clamp_angle(ang)

            # publish to motor loop
            global latest_target_angle, latest_target_time
            latest_target_angle = ang
            latest_target_time  = time.time()
    return handler

async def connect_device(device):
    addr = device.address
    name = device.name or addr
    suf = nicla_suffix(name)
    if suf not in ALLOWED_SUFFIXES:
        print(f"⏭️  Not allowed: {name} [{addr}] (suffix={suf})")
        return False
    if addr in clients and clients[addr].is_connected:
        return True
    client = BleakClient(device, disconnected_callback=lambda c: on_disconnect(device))
    try:
        await client.connect(timeout=10.0)
        print(f"✅ Connected to {name} [{addr}] (suffix={suf})")
        await client.start_notify(CHAR_UUID, make_handler(name, addr))
        clients[addr] = client
        return True
    except Exception as e:
        print(f"❌ Connect failed {name} [{addr}]: {e}")
        return False

def on_disconnect(device):
    addr = device.address
    name = device.name or addr
    print(f"⚠️ Disconnected: {name} [{addr}]")
    clients.pop(addr, None)
    latest_euler.pop(addr, None)

async def scan_targets():
    print("🔍 Scanning for devices…")
    found = await BleakScanner.discover(timeout=5.0)
    def has_service(d):
        uu = d.metadata.get("uuids") or []
        return SERVICE_UUID in {u.lower() for u in uu}
    filtered = []
    for d in found:
        if not has_service(d):
            continue
        suf = nicla_suffix(d.name or "")
        if suf in ALLOWED_SUFFIXES:
            filtered.append(d)
        else:
            if d.name:
                print(f"⏭️  Skipping {d.name} [{d.address}] (suffix={suf})")
            else:
                print(f"⏭️  Skipping [{d.address}] (no name)")
    return filtered

# ===================== EPOS WATCHDOG (no recalibration) =====================
async def epos_watchdog():
    while True:
        try:
            if keyHandle and get_fault():
                print("[WATCHDOG] EPOS FAULT → clearing & re-arming…")
                async with EPOS_LOCK:
                    try:
                        halt()
                        clear_fault_and_reenable()
                    except Exception:
                        pass
                    try:
                        if POS_MIN is not None and POS_MAX is not None:
                            guard = 500
                            cur = get_position()
                            low, high = POS_MIN - guard, POS_MAX + guard
                            if cur < low:
                                tgt = POS_MIN + 100
                                move_to_position_with_guard(tgt, target_speed=1500, accel=1500, decel=1500, timeout_s=1.2)
                            elif cur > high:
                                tgt = POS_MAX - 100
                                move_to_position_with_guard(tgt, target_speed=1500, accel=1500, decel=1500, timeout_s=1.2)
                    except Exception:
                        pass
        except Exception:
            pass
        await asyncio.sleep(0.005)

# ===================== MOTOR CONTROL LOOP =====================
async def motor_control_loop():
    global last_angle_cmd, last_cmd_time
    while True:
        try:
            now = time.time()
            if (latest_target_angle is not None and
                (now - latest_target_time) < 1.0 and
                POS_MIN is not None and POS_MAX is not None):
                ang = latest_target_angle
                step = abs(ang - last_angle_cmd)
                if step >= 0.5 or (now - last_cmd_time) > 0.002:
                    sp = 12000; ac = 30000; dc = 30000
                    target_pos = angle_to_position(ang)
                    #print(f"Angle {ang:.1f}°")
                    async with EPOS_LOCK:
                        safe_move(target_pos, speed=sp, accel=ac, decel=dc)
                    last_angle_cmd = ang
                    last_cmd_time  = now
            await asyncio.sleep(0.001)
        except Exception:
            await asyncio.sleep(0.005)

# ===================== MAIN =====================
async def main():
    global keyHandle, POS_MIN, POS_MAX
    print("Starting (Nicla→Angle→Motor)…")

    # EPOS open/enable
    keyHandle = epos.VCS_OpenDevice(b'EPOS4', b'CANopen', b'CAN_mcp251xfd 0', b'CAN0', byref(pErrorCode))
    if not keyHandle:
        raise RuntimeError(f"OpenDevice NULL: 0x{pErrorCode.value:08X} – {get_err_text(pErrorCode.value)}")
    CHECK(epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode)),
          "SetProtocolStackSettings", pErrorCode)
    CHECK(epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode)), "ClearFault", pErrorCode)
    CHECK(epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode)),
          "ActivateProfilePositionMode", pErrorCode)
    CHECK(epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode)), "SetEnableState", pErrorCode)

    # Load limits or MAX-only calibrate
    loaded = load_limits()
    first_time=True
    if first_time==False:
        POS_MIN, POS_MAX = loaded
        print(f"[LIMITS] Loaded: MIN={POS_MIN}, MAX={POS_MAX}, SPAN={POS_MAX - POS_MIN}")
    else:
        # decide span
        span = DESIRED_SPAN_STEPS
        if span is None:
            # if we previously saved a file, we'd have loaded it above.
            # so this is truly first time → use default
            span = DEFAULT_SPAN_STEPS
        POS_MIN, POS_MAX, pos_center = auto_calibrate_max_only(
            step=1000, speed=5000, desired_span=span
        )
        print(f"[LIMITS] Calibrated(MAX-only): MIN={POS_MIN}, MAX={POS_MAX}, CENTER={pos_center}")
        save_limits(POS_MIN, POS_MAX)
        first_time=False
        move_to_position_with_guard(pos_center, target_speed=2500, accel=2500, decel=2500, timeout_s=1.2)

    # BLE discover/connect (only allowed suffixes)
    devices = await scan_targets()
    if not devices:
        print("⚠️  No Nicla devices found (advertising your service).")
    else:
        await asyncio.gather(*[connect_device(d) for d in devices])

    # Start watchdog + motor control
    wd_task  = asyncio.create_task(epos_watchdog())
    ctl_task = asyncio.create_task(motor_control_loop())

    # Run until Ctrl-C
    loop = asyncio.get_event_loop()
    stop_future = loop.create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_future.set_result, None)
    await stop_future

    # Cleanup
    ctl_task.cancel(); wd_task.cancel()
    for c in list(clients.values()):
        try: await c.disconnect()
        except Exception: pass

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
