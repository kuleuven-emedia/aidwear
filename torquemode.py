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



#torque/current mode 


epos.VCS_ActivateCurrentMode.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]
epos.VCS_SetCurrentMust.argtypes = [c_void_p, c_ushort, c_short, POINTER(c_uint)]



def activate_current_mode():
    ec = c_uint()
    CHECK(
        epos.VCS_ActivateCurrentMode(keyHandle, nodeID, byref(ec)),
        "ActivateCurrentMode", ec
    )
    
KT = 0.0276       # Nm/A
I_MAX_mA = 3000   # limite massimo corrente

def set_current_mA(tau_des):
    """
    Converte il torque desiderato [Nm] in corrente [mA] 
    e invia il comando all'EPOS.
    """
    ec = c_uint()
    current_mA = (tau_des / KT) * 1000  # Nm → mA

    # saturazione sicurezza
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

    # >>> Stay in torque mode 
    CHECK(epos.VCS_ActivateCurrentMode(keyHandle, nodeID, byref(ec)),
          "ActivateCurrentMode(clear)", ec)


    CHECK(epos.VCS_SetCurrentMust(keyHandle, nodeID,
          c_uint(int(KT)), c_uint(int(I_MAX_mA)), byref(ec)),
          "SetCurrentMust(clear)", ec)

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

# Global limits
POS_MIN = None
POS_MAX = None


    
def angle_to_position(angle_deg: float) -> int:
    """Converte un angolo (°) in posizione encoder, restando nel range POS_MIN..POS_MAX."""
    a = clamp_angle(float(angle_deg))
    frac = (a - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN)
    frac = max(0.0, min(1.0, frac))  # clamp di sicurezza software
    span = POS_MAX - POS_MIN
    return int(round(POS_MIN + frac * span))

    


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
        except Exception:
            pass
        await asyncio.sleep(0.005)



# ===================== TORQUE CONTROL LOOP (NO AUTOTUNING) =====================

# guadagni fissi
Kp_deg_to_torque = 0.005   # Nm per grado di errore
Kd_deg_to_torque = 0.0008  # Nm per (deg/s)
TORQUE_MAX = 0.5          # Nm massimo
ANGLE_DEADBAND = 0.5 #0.5      # deg, sotto questo → torque 0

last_ang_cur = 0.0

async def motor_control_loop_torque():
    global last_angle_cmd, last_cmd_time, last_ang_cur
    global Kp_deg_to_torque, Kd_deg_to_torque

    dt = 0.05  # tempo ciclo ~20Hz

    while True:
        try:
            now = time.time()

            # se abbiamo un target recente e limiti validi
            if (latest_target_angle is not None and
                (now - latest_target_time) < 1.0 and
                POS_MIN is not None and POS_MAX is not None):

                # setpoint desiderato
                ang_des = float(latest_target_angle)

                # lettura posizione corrente
                pos = get_position()
                ang_cur = position_to_angle(pos)

                # errore
                err = ang_des - ang_cur

                # deadband
                if abs(err) < ANGLE_DEADBAND:
                    set_current_mA(0)
                    last_angle_cmd = ang_des
                    last_cmd_time = now
                    await asyncio.sleep(dt)
                    continue

                # velocità angolare stimata (derivata numerica)
                vel_cur = (ang_cur - last_ang_cur) / dt
                last_ang_cur = ang_cur

                # controllo PD → torque richiesto
                tau_des = (Kp_deg_to_torque * err) - (Kd_deg_to_torque * vel_cur)

                # limiti
                tau_des = max(-TORQUE_MAX, min(TORQUE_MAX, tau_des))

                # invia al motore
                set_current_mA(tau_des)

                last_angle_cmd = ang_des
                last_cmd_time  = now

            else:
                # nessun target attivo → ferma motore
                set_current_mA(0)

            await asyncio.sleep(dt)

        except Exception:
            await asyncio.sleep(dt)





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
    CHECK(epos.VCS_ActivateCurrentMode(keyHandle, nodeID, byref(pErrorCode)),
    "ActivateCurrentMode", pErrorCode)
 
    KT = 0.0276       # Nm/A
    I_MAX_mA = 3000   # limite massimo corrente


    # BLE discover/connect (only allowed suffixes)
    devices = await scan_targets()
    if not devices:
        print("⚠️  No Nicla devices found (advertising your service).")
    else:
        await asyncio.gather(*[connect_device(d) for d in devices])

    # Start watchdog + motor control
    wd_task  = asyncio.create_task(epos_watchdog())
    ctl_task = asyncio.create_task(motor_control_loop_torque())

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
