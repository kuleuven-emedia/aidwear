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



#Velocity mode 


epos.VCS_ActivateProfileVelocityMode.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]
epos.VCS_MoveWithVelocity.argtypes            = [c_void_p, c_ushort, c_long, POINTER(c_uint)]

epos.VCS_SetVelocityProfile.argtypes = [c_void_p, c_ushort, c_uint, c_uint, POINTER(c_uint)]


def set_velocity(vel_counts_per_s: int):
    """Positive = move toward POS_MAX, negative = toward POS_MIN."""
    ec = c_uint()
    epos.VCS_MoveWithVelocity(keyHandle, nodeID, c_long(int(vel_counts_per_s)), byref(ec))


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
ANGLE_MIN = 0.0
ANGLE_MAX = 120
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

    move_to_position_with_guard(pos_min, target_speed=2500, accel=2500, decel=2500, timeout_s=1.2)

    # >>> Stay in VELOCITY mode (not position)
    CHECK(epos.VCS_ActivateProfileVelocityMode(keyHandle, nodeID, byref(ec)),
          "ActivateProfileVelocityMode(clear)", ec)

    # Raise the velocity ramp (counts/s^2). Tune to your mechanics.
    vel_accel = 100000   # try 60k → 100k → 150k


    CHECK(epos.VCS_SetVelocityProfile(keyHandle, nodeID,
          c_uint(int(vel_accel)), c_uint(int(vel_decel)), byref(ec)),
          "SetVelocityProfile(clear)", ec)

    time.sleep(0.002)


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
'''
DESIRED_SPAN_STEPS = 200000   # e.g. 303047  (set to an int to force)
DEFAULT_SPAN_STEPS = 200000 # fallback if nothing saved and DESIRED_SPAN_STEPS is None



def auto_calibrate_limits(step=1000,
                          speed=default_speed, accel=acceleration, decel=deceleration,
                          settle_backoff=1500, min_span=3000):
    start = get_position()
    print(f"[CAL] start={start}")
    pos_max, why_p = creep_scan(+1, step=step, speed=speed, accel=accel, decel=decel)
    print(f"[CAL] pos_max={pos_max} ({why_p})")
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

def auto_calibrate_min_only(step=1000,
                            speed=default_speed, accel=acceleration, decel=deceleration,
                            settle_backoff=1500,
                            desired_span=None):
    
    cur = get_position()
    print(f"[CAL] (MIN-only) start={cur}")

    pos_min, why_p = creep_scan(-1, step=step, speed=speed, accel=accel, decel=decel)
    print(f"[CAL] pos_max={pos_min} ({why_p})")

    

    # Decide span
    span = int(desired_span)
    if span <= 0:
        raise ValueError("desired_span must be positive")

    s_min = to_signed32(pos_min)
    s_max = s_min + span
    center = s_min + s_max 
    print(f"[CAL] span={span} → MIN={s_min}, MAX={s_max}, center={center}")
    return s_min, s_max, center



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
'''
# Global limits
POS_MIN = None
POS_MAX = None

'''
def angle_to_position(angle_deg: float) -> int:
    a = clamp_angle(float(angle_deg))
    frac = (a - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN)
    span = POS_MAX - POS_MIN
    return int(round(POS_MIN + frac * span))
   '''
    
def angle_to_position(angle_deg: float) -> int:
    """Converte un angolo (°) in posizione encoder, restando nel range POS_MIN..POS_MAX."""
    a = clamp_angle(float(angle_deg))
    frac = (a - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN)
    frac = max(0.0, min(1.0, frac))  # clamp di sicurezza software
    span = POS_MAX - POS_MIN
    return int(round(POS_MIN + frac * span))

    
'''
def position_to_angle(pos: int) -> float:
    span = float(POS_MAX - POS_MIN)
    if span <= 0:
        return ANGLE_MIN
    frac = (float(pos) - float(POS_MIN)) / span
    #ang  = ANGLE_MIN + frac * (ANGLE_MAX - ANGLE_MIN)
    ang = frac*90
    #return clamp_angle(ang)
    return ang
    '''

'''
la mia usata prima

def position_to_angle(pos: int) -> float:
    """Mappa pos (counts) → angolo reale (deg) usando ANGLE_MIN/ANGLE_MAX."""
    span = float(POS_MAX - POS_MIN)
    if span <= 0:
        return ANGLE_MIN
    frac = (float(pos) - float(POS_MIN)) / span
    ang = ANGLE_MIN + frac * (ANGLE_MAX - ANGLE_MIN)
    return clamp_angle(ang)
'''

def position_to_angle(pos: int) -> float:
    """Converte posizione encoder in angolo logico (°)."""
    span = float(POS_MAX - POS_MIN)
    if span <= 0:
        return ANGLE_MIN
    frac = (float(pos) - float(POS_MIN)) / span
    frac = max(0.0, min(1.0, frac))  # sicurezza
    ang = ANGLE_MIN + frac * (ANGLE_MAX - ANGLE_MIN)
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

        # NEW: also store by suffix so we can force ordering: 6845 first, e0c6 second
        suf = nicla_suffix(name) or ""
        if suf:
            latest_euler_by_suffix[suf] = (heading, pitch, roll)

        # Compute only when we have BOTH
        if "6845" in latest_euler_by_suffix and "e0c6" in latest_euler_by_suffix:
            e_6845 = latest_euler_by_suffix["6845"]
            e_e0c6 = latest_euler_by_suffix["e0c6"]

            # pitch is index 1
            raw_ang = float(e_6845[1] - e_e0c6[1])   # <-- ORDER IS FIXED HERE

            # clamp to 0..90° then to your allowed band
            #raw_ang = max(-5.0, min(95.0, raw_ang))  la mia usata prima
            # sicurezza "morbida" anti-spike (opzionale)
            raw_ang = max(ANGLE_MIN - 5, min(ANGLE_MAX + 5, raw_ang))
            #raw_ang = clamp_angle(ang)
            
            # Check if in allowed range
            if ANGLE_MIN <= raw_ang <= ANGLE_MAX:
                # Valid → use it
                ang = clamp_angle(raw_ang)  # This is still safe
                print(f"[VALID] Angle {ang:.2f}")
                global latest_target_angle, latest_target_time
                latest_target_angle = ang
                latest_target_time  = time.time()
            else:
                # Invalid → ignore update
                print(f"[OUT-OF-RANGE] Ignored angle {raw_ang:.2f} (not in [{ANGLE_MIN}, {ANGLE_MAX}])")
            '''
            print(f"Angle {ang}")
            # publish to motor loop
            global latest_target_angle, latest_target_time
            latest_target_angle = ang
            latest_target_time  = time.time()
            '''
            #print(f"[DEBUG] Raw angle diff: {e_6845[1] - e_e0c6[1]:.2f} → clamped: {ang:.2f}")
            print(f"[DEBUG] Pitches: 6845={e_6845[1]:.2f}, e0c6={e_e0c6[1]:.2f}, diff={ang:.2f}")

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
                print(f"Error angle {position_to_angle(get_position())}")
                async with EPOS_LOCK:
                    try:
                        halt()
                        clear_fault_and_reenable()
                        print(f"Error angle Lock{position_to_angle(get_position())}")
                    except Exception:
                        pass
                    """ try:
                        if POS_MIN is not None and POS_MAX is not None:
                            guard = 500
                            cur = get_position()
                            low, high = POS_MIN + guard, POS_MAX - guard
                            if cur < low:
                                tgt = POS_MIN + 1000
                                move_to_position_with_guard(tgt, target_speed=1500, accel=1500, decel=1500, timeout_s=1.2)
                            elif cur > high:
                                tgt = POS_MAX - 1000
                                move_to_position_with_guard(tgt, target_speed=1500, accel=1500, decel=1500, timeout_s=1.2)
                    except Exception:
                        pass  """
        except Exception:
            pass
        await asyncio.sleep(0.005)

# ===================== MOTOR CONTROL LOOP =====================
# ---- tuning knobs for smooth feel ----
Kp_deg_to_counts_per_s = 1650.0   # counts/s per degree of error (tweak)  MODIFICATO
VEL_MAX = 10000                   # max |velocity| in counts/s (tweak)
ANGLE_DEADBAND = 0.5             # deg, below this → command 0
EDGE_SLOWDOWN_DEG = 0.0           # start slowing near ends to avoid hard hits

async def motor_control_loop():
    global last_angle_cmd, last_cmd_time
    steps_per_deg = float(POS_MAX - POS_MIN) / (ANGLE_MAX - ANGLE_MIN)

    while True:
        try:
            now = time.time()
            if (latest_target_angle is not None and
                (now - latest_target_time) < 1.0 and
                POS_MIN is not None and POS_MAX is not None):

                # desired angle (already clamped in your handler)
                ang_des = float(latest_target_angle)

                # read current position and map to angle
                pos = get_position()
                ang_cur = position_to_angle(pos)

                # error and deadband
                err = ang_des - ang_cur
                if abs(err) < ANGLE_DEADBAND:# if the variatoin of angle is lower then the deadband do not move it
                    set_velocity(0)
                    last_angle_cmd = ang_des
                    last_cmd_time  = now
                    await asyncio.sleep(0.05)
                    continue

                # basic P → velocity (counts/s)
                vel_cmd = Kp_deg_to_counts_per_s * err #Double check this ?????
                # si assicura che che il segno sia rispettato
                if 0 < abs(vel_cmd) < 1000:
                    vel_cmd = 1000 * (1 if vel_cmd > 0 else -1)
                vel_cmd = int(max(-VEL_MAX, min(VEL_MAX, vel_cmd)))

                # slow down when close to software ends
                # (optional but helps smoothness)
                margin_low  = (ang_cur - ANGLE_MIN)
                margin_high = (ANGLE_MAX - ang_cur)
                if (vel_cmd < 0) and (margin_low < EDGE_SLOWDOWN_DEG):
                    vel_cmd *= max(0.2, margin_low / EDGE_SLOWDOWN_DEG)
                if (vel_cmd > 0) and (margin_high < EDGE_SLOWDOWN_DEG):
                    vel_cmd *= max(0.2, margin_high / EDGE_SLOWDOWN_DEG)

                # clamp and send
                vel_cmd = int(max(-VEL_MAX, min(VEL_MAX, vel_cmd)))
                set_velocity(vel_cmd)

                last_angle_cmd = ang_des
                last_cmd_time  = now

            else:
                # no fresh target → stop softly
                set_velocity(0)

            await asyncio.sleep(0.05)  # ~500 Hz command rate
        except Exception:
            # never crash loop
            await asyncio.sleep(0.05)


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

    # ===================== EPOS LIMITS SETUP (STATIC) =====================
    # Imposta qui i valori fissi che conosci già
    POS_MIN = -204011       # <-- metti il tuo valore reale
    POS_MAX = 150464      # <-- metti il tuo valore reale
    POS_CENTER = -21744

    print(f"[LIMITS] Using fixed values: MIN={POS_MIN}, MAX={POS_MAX}, CENTER={POS_CENTER}")

    # (opzionale) salva su file per compatibilità futura
    save_limits(POS_MIN, POS_MAX)

    # Porta subito il motore in posizione centrale all'avvio
    move_to_position_with_guard(
        POS_CENTER,
        target_speed=2500,
        accel=2500,
        decel=2500,
        timeout_s=1.2
    )

        
    #We are switching to velocity mode 
    CHECK(epos.VCS_ActivateProfileVelocityMode(keyHandle, nodeID, byref(pErrorCode)),
    "ActivateProfileVelocityMode", pErrorCode)
    CHECK(epos.VCS_ActivateProfileVelocityMode(keyHandle, nodeID, byref(pErrorCode)),
      "ActivateProfileVelocityMode", pErrorCode)

    vel_accel = 100000
    vel_decel = 100000
    CHECK(epos.VCS_SetVelocityProfile(keyHandle, nodeID,
    c_uint(int(vel_accel)), c_uint(int(vel_decel)), byref(pErrorCode)),
        "SetVelocityProfile", pErrorCode)

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
