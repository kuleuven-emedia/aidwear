#!/usr/bin/env python3
import time
from ctypes import *
import ctypes.util
import asyncio, struct, signal
import numpy as np
from scipy.spatial.transform import Rotation
from bleak import BleakScanner, BleakClient

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
    time.sleep(0.05)

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
        time.sleep(0.01)

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
        # stall/timeout/fault → stop here
        return p, why
    return pos, "range"

def auto_calibrate_limits(step=1000,
                          speed=default_speed, accel=acceleration, decel=deceleration,
                          settle_backoff=1500, min_span=3000):
    start = get_position()
    print(f"[CAL] start={start}")
    pos_max, why_p = creep_scan(+1, step=step, speed=speed, accel=accel, decel=decel)
    print(f"[CAL] pos_max={pos_max} ({why_p})")
    # come back a little
    _p2, _ = move_to_position_with_guard(
        target_position=pos_max - settle_backoff,
        target_speed=max(2000, speed//2),
        accel=accel, decel=decel,
        timeout_s=1.0
    )
    time.sleep(0.2)
    pos_min, why_m = creep_scan(-1, step=step, speed=speed, accel=accel, decel=decel)
    print(f"[CAL] pos_min={pos_min} ({why_m})")

    s_min = to_signed32(pos_min)
    s_max = to_signed32(pos_max)
    if s_min > s_max:
        s_min, s_max = s_max, s_min
    span   = s_max - s_min
    center = s_min + span // 2
    print(f"[CAL] span={span} → center={center}")
    if span < min_span:
        print("⚠️  Very small span – check EPOS limits.")
    return s_min, s_max, center

# Global limits (filled after calibration)
POS_MIN = None
POS_MAX = None

def angle_to_position(angle_deg: float) -> int:
    """Map 0..90° → POS_MIN..POS_MAX."""
    angle = max(0.0, min(90.0, float(angle_deg)))
    span = POS_MAX - POS_MIN
    return int(round(POS_MIN + (angle / 90.0) * span))

def safe_move(target_signed_pos: int, speed=3000, accel=3000, decel=3000):
    tgt = max(POS_MIN, min(POS_MAX, int(target_signed_pos)))
    pos, why = move_to_position_with_guard(
        target_position=tgt,
        target_speed=speed, accel=accel, decel=decel
    )
    # You can uncomment to watch moves:
    # print(f"[MOVE] target={target_signed_pos} (clamped {tgt}) -> pos={pos} [{why}]")
    return pos, why

# ===================== BLE / ANGLE SOURCING =====================

SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214".lower()
CHAR_UUID    = "19b10002-e8f2-537e-4f6c-d104768a1214".lower()  # ts + gyro + euler (28 bytes)

clients = {}            # addr -> BleakClient
latest_euler = {}       # addr -> (heading, pitch, roll)
rx_counts = {}          # addr -> count
last_angle_cmd = 0.0    # last commanded angle
last_cmd_time  = 0.0

# shared target angle: computed from two Niclas
latest_target_angle = None
latest_target_time  = 0.0

def angle_between_eulers(e1, e2):
    """Overall 3D angle between two orientations (degrees)."""
    try:
        r1 = Rotation.from_euler('zyx', e1, degrees=True)
        r2 = Rotation.from_euler('zyx', e2, degrees=True)
        #print(f"{r1}")
        r = r1.inv() * r2
        return np.linalg.norm(r.as_rotvec()) * 180.0 / np.pi
    except Exception:
        return None

def make_handler(name, addr):
    def handler(_, data: bytearray):
        nonlocal addr
        if len(data) != 28:
            return
        try:
            # NOTE: Arduino sends (ts, gx, gy, gz, pitch, roll, heading) as <I6f
            ts, gx, gy, gz, pitch, roll, heading = struct.unpack("<I6f", data)
        except struct.error:
            return

        # Store current euler as (heading, pitch, roll)
        latest_euler[addr] = (heading, pitch, roll)
        rx_counts[addr] = rx_counts.get(addr, 0) + 1

        # If we have at least two devices, compute angle and store it
        if len(latest_euler) >= 2:
            addrs = list(latest_euler.keys())
            # just take the first pair
            a1, a2 = addrs[0], addrs[1]
            e1, e2 = latest_euler[a1], latest_euler[a2]
            #ang = angle_between_eulers(e1, e2)
            ang = e1[1] - e2[1]
            #print(f"{e1[1]:.1f},{e2[1]:.1f},angle:{ang:.1f}°")
            
            if ang is not None:
                # clamp 0..90 and publish to motor loop
                ang = max(0.0, min(90.0, float(ang)))
                # smoothing/low-pass could be added here if needed
                global latest_target_angle, latest_target_time
                latest_target_angle = ang
                latest_target_time  = time.time()
    return handler

async def connect_device(device):
    addr = device.address
    name = device.name or addr
    if addr in clients and clients[addr].is_connected:
        return True
    client = BleakClient(device, disconnected_callback=lambda c: on_disconnect(device))
    try:
        await client.connect(timeout=10.0)
        print(f"✅ Connected to {name} [{addr}]")
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
    # NOTE: Bleak on Linux: device.metadata["uuids"] may exist, or not.
    def has_service(d):
        uu = d.metadata.get("uuids") or []
        return SERVICE_UUID in {u.lower() for u in uu}
    return [d for d in found if has_service(d)]

# ===================== MOTOR CONTROL LOOP (drives EPOS) =====================

async def motor_control_loop():
    """
    Polls latest_target_angle and sends position commands at ~10 Hz.
    Ignores stale angle (>1.0 s old) or tiny changes (<0.5°).
    """
    global last_angle_cmd, last_cmd_time
    while True:
        try:
            now = time.time()
            if latest_target_angle is not None and (now - latest_target_time) < 1.0:
                ang = latest_target_angle
                # throttle updates if change is tiny
                if abs(ang - last_angle_cmd) >= 0.5 or (now - last_cmd_time) > 0.01:
                    print(f"Angle {ang:.1f}°")
                    target_pos = angle_to_position(ang)
                    safe_move(target_pos, speed=12000, accel=30000, decel=30000)
                    last_angle_cmd = ang
                    last_cmd_time  = now
            await asyncio.sleep(0.001)  # 0.01
        except Exception:
            # don’t crash the loop on random EPOS hiccups
            await asyncio.sleep(0.02) #0.2

# ===================== MAIN =====================

async def main():
    global keyHandle, POS_MIN, POS_MAX
    print("Starting (Nicla→Angle→Motor)…")

    # ---- EPOS open/enable
    keyHandle = epos.VCS_OpenDevice(b'EPOS4', b'CANopen', b'CAN_mcp251xfd 0', b'CAN0', byref(pErrorCode))
    if not keyHandle:
        raise RuntimeError(f"OpenDevice NULL: 0x{pErrorCode.value:08X} – {get_err_text(pErrorCode.value)}")

    CHECK(epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode)),
          "SetProtocolStackSettings", pErrorCode)
    CHECK(epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode)), "ClearFault", pErrorCode)
    CHECK(epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode)),
          "ActivateProfilePositionMode", pErrorCode)
    CHECK(epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode)), "SetEnableState", pErrorCode)

    # ---- Calibrate once
    POS_MIN, POS_MAX, pos_center = auto_calibrate_limits(step=1000, speed=5000)
    print(f"[LIMITS] MIN={POS_MIN}, MAX={POS_MAX}, CENTER={pos_center}")
    safe_move(pos_center, speed=2500)

    # ---- BLE discover/connect
    devices = await scan_targets()
    if not devices:
        print("⚠️  No Nicla devices found (advertising your service).")
    else:
        await asyncio.gather(*[connect_device(d) for d in devices])

    # ---- Start motor control loop
    ctl_task = asyncio.create_task(motor_control_loop())

    # Run until Ctrl-C
    loop = asyncio.get_event_loop()
    stop_future = loop.create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_future.set_result, None)
    await stop_future

    # Cleanup
    ctl_task.cancel()
    for c in list(clients.values()):
        try:
            await c.disconnect()
        except Exception:
            pass

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
