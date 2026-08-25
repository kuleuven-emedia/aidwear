"""
Filename: scripts/start_pmu.py
Author: Elias Thiery <ethiery@vub.be>
Date: 2025-01-02
Version: 1.0
Description: Pre-launch script to allocate a predefined CAN ID
    to the MatekSys power monitoring unit via the DroneCAN protocol.
"""

import os
import time
import threading
import can
import dronecan

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

    node = dronecan.make_node("can0", node_id=11, bitrate=1000000)

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


now = time.time()

while time.time() - now < 3:
    # print(f"vol: {battery_voltage}, cur: {battery_current}")
    time.sleep(0.01)
