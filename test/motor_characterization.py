import os
import time
import csv
import threading
import gpiod
import can
import dronecan

from cubemars_can_commands import *

os.system("sudo ip link set can0 down")
os.system("sudo ip link set can1 down")
os.system("sudo ip link set can0 up type can bitrate 1000000")
os.system("sudo ip link set can1 up type can bitrate 1000000")
os.system("sudo ifconfig can0 txqueuelen 65536")
os.system("sudo ifconfig can1 txqueuelen 65536")

can0 = can.interface.Bus(channel="can0", interface="socketcan")

battery_voltage = 0.0
battery_current = 0.0
battery_power = 0.0


def start_dronecan_battery_reader():
    import can

    def _flush_tx_buffer(self):
        return None

    can.BusABC.flush_tx_buffer = _flush_tx_buffer

    node = dronecan.make_node("can1", node_id=11, bitrate=1000000)

    node_monitor = dronecan.app.node_monitor.NodeMonitor(node)
    dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(
        node, node_monitor
    )

    def on_battery(event):
        global battery_voltage, battery_current, battery_power
        msg = event.message
        battery_voltage = msg.voltage
        battery_current = msg.current
        battery_power = battery_voltage * battery_current

    node.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, on_battery)

    print("[DroneCAN] Battery reader thread started.")

    try:
        node.spin()
    except KeyboardInterrupt:
        pass


battery_thread = threading.Thread(target=start_dronecan_battery_reader, daemon=True)
battery_thread.start()

K = {"AK10-9": [0.5, 0.06, 0], "AK80-8": [0.1, 0.01, 0]}
motor_type = "AK80-8"
# motor_id = 2
motor_id = 1
torques = range(0, 19)
# torques = range(45,-46,-1)
# torques = [55] * 10

from gpiod.line import Direction, Value


def get_line_value(chip_path, line_offset):
    with gpiod.request_lines(
        chip_path,
        consumer="get-line-value",
        config={line_offset: gpiod.LineSettings(direction=Direction.INPUT)},
    ) as request:
        value = request.get_value(line_offset)
        return value


with open("unsaved_data.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(
        [
            "timestamp_ms",
            "motor_current_A",
            "motor_velocity_rpm",
            "motor_temperature_C",
            "battery_voltage_V",
            "battery_current_A",
            "battery_power_W",
        ]
    )

    while get_line_value("/dev/gpiochip4", 26) != Value.ACTIVE:
        pin = get_line_value("/dev/gpiochip4", 26)
        print(f"waiting for Beckhoff, pin input = {pin}")
        time.sleep(0.002)

    print("start")
    start = time.time()

    for torque in torques:
        if torque == torques[0]:
            lim = 50
        else:
            lim = 50
        for samples in range(lim):
            comm_can_set_torque(can0, motor_id, torque, motor_type)
            print(f"torque: {torque} Nm")

            while not (can0.recv(timeout=0) is None):
                pass

            for msg0 in can0:
                motor_data = on_message_received(motor_id, msg0, motor_type)
                if motor_data is not None:
                    break

            motor_pos, motor_spd, motor_cur, motor_temp, motor_err, _ = motor_data

            bv = battery_voltage
            bc = battery_current
            bp = battery_power

            timestamp_ms = int((time.time() - start) * 1000)

            writer.writerow(
                [timestamp_ms, motor_cur, motor_spd, motor_temp, bv, bc, bp]
            )

comm_can_set_torque(can0, motor_id, 0, motor_type)

print("Test complete.")
