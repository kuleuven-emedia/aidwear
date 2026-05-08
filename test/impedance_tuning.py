import threading
import time
import can

from hermes.aidwear.can_control.motor_cubemars import can_set_position_impedance

# Motor config: motor_id -> motor_type
MOTOR_IDS = [1, 2, 3, 4]
MOTOR_TYPES = {1: "AK10-9", 2: "AK80-8", 3: "AK80-8", 4: "AK10-9"}

# Separate gain dictionaries per motor type
GAINS = {"AK10-9": [0.5, 0.06, 0.0], "AK80-8": [0.1, 0.01, 0.0]}  # [Kp, Kv, Ka]


def control_loop(running: bool, bus: list[can.BusABC]):
    ref = [0, 0, 0]  # [pos, vel, acc] setpoints
    while running:
        for motor_id in MOTOR_IDS:
            mtype = MOTOR_TYPES[motor_id]
            K = GAINS[mtype]
            can_set_position_impedance(
                bus=bus[0], controller_id=motor_id, ref=ref, K=K, motor_type=mtype
            )
        time.sleep(0.02)  # 50 Hz


def input_thread(running: bool):
    while running:
        if keyboard.is_pressed("g"):
            try:
                print("\nEnter motor type to update (AK10-9 or AK80-8):")
                mtype = input().strip().upper()
                mtype_map = {
                    k.upper(): k for k in GAINS
                }  # map uppercase input to original key
                if mtype not in mtype_map:
                    print(
                        f"❌ Unknown motor type: {mtype}. Available types: {list(GAINS.keys())}"
                    )
                    continue
                mtype_key = mtype_map[mtype]
                print(f"Current gains for {mtype}: {GAINS[mtype]}")
                print("Enter new gains (Kp Kv Ka), e.g., 0.6 0.02 0:")
                raw = input()
                parts = [float(x) for x in raw.strip().split()]
                if len(parts) == 3:
                    GAINS[mtype] = parts
                    print(f"✅ Updated gains for {mtype}: {GAINS[mtype]}")
                else:
                    print("❌ Please enter exactly 3 values.")
            except Exception as e:
                print("❌ Error:", e)
            time.sleep(0.5)  # debounce
        if keyboard.is_pressed("q"):
            print("Exiting...")
            running = False
            break
        time.sleep(0.1)


if __name__ == "__main__":
    running = True
    # Replace with actual bus setup
    bus0 = can.interface.Bus(channel="can0", interface="socketcan")
    bus = [bus0]

    # Start control and input threads
    control_thread = threading.Thread(
        target=control_loop, args=(running, bus), daemon=True
    )  # NOTE: @EliasThiery why this has to run as a daemon?
    control_thread.start()
    input_thread(running)
    control_thread.join()
