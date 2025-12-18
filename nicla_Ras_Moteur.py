#!/usr/bin/env python3
import asyncio
import struct
import signal
import time

import numpy as np
from scipy.spatial.transform import Rotation
from bleak import BleakScanner, BleakClient

# ====== CONFIG ======
SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214".lower()
CHAR_UUID    = "19b10002-e8f2-537e-4f6c-d104768a1214".lower()  # single stream: ts + gyro + euler

PRINT_EVERY_N = 20     # print every N packets per device
REPORT_EVERY_S = 5.0   # print relative-angles report every N seconds

# ====== RUNTIME STATE ======
clients = {}              # addr -> BleakClient
rx_counts = {}            # addr -> number of packets received
latest_euler = {}         # addr -> (heading, pitch, roll)
last_report_t = 0.0

acceleration = 30000 # rpm/s, up to 1e7 would be possible
deceleration = 30000 # rpm/s

# ====== MATH HELPERS ======
def angle_between_eulers(e1, e2):
    """Overall 3D angle between two orientations (degrees)."""
    try:
        r1 = Rotation.from_euler('zyx', e1, degrees=True)
        r2 = Rotation.from_euler('zyx', e2, degrees=True)
        r = r1.inv() * r2
        return np.linalg.norm(r.as_rotvec()) * 180.0 / np.pi
    except Exception:
        return None

def per_axis_diffs(e1, e2):
    """Shortest per-axis differences (wrap-safe, degrees)."""
    (h1, p1, r1), (h2, p2, r2) = e1, e2
    def d(a, b):
        diff = (a - b) % 360.0
        return 360.0 - diff if diff > 180.0 else diff
    return {"heading": d(h1, h2), "pitch": d(p1, p2), "roll": d(r1, r2)}

# ====== BLE HANDLER ======
def make_handler(name, addr):
    def handler(_, data: bytearray):
        global last_report_t

        if len(data) != 28:
            print(f"[{name}] ⚠️ Unexpected packet size: {len(data)}")
            return

        try:
            # Parse packet: ts + gx + gy + gz + pitch + roll + heading
            ts, gx, gy, gz, pitch, roll, heading = struct.unpack("<I6f", data)
        except struct.error as e:
            print(f"[{name}] Parse error: {e} (len={len(data)})")
            return

        # Update stats & state
        rx_counts[addr] = rx_counts.get(addr, 0) + 1
        latest_euler[addr] = (heading, pitch, roll)

        # Print periodically
        if rx_counts[addr] % PRINT_EVERY_N == 0:
            print(f"[{name}] {ts:6d} ms | "
                  f"G=({gx:.2f},{gy:.2f},{gz:.2f}) | "
                  f"Euler=({pitch:.1f},{roll:.1f},{heading:.1f}) | "
                  f"rx={rx_counts[addr]}")

        # Periodic relative-angle report (if 2+ devices)
        now = time.time()
        if now - last_report_t >= REPORT_EVERY_S and len(latest_euler) >= 2:
            last_report_t = now
            addrs = list(latest_euler.keys())
            print("=== Relative Angles Between IMUs ===")
            for i in range(len(addrs)):
                for j in range(i + 1, len(addrs)):
                    a1, a2 = addrs[i], addrs[j]
                    e1, e2 = latest_euler[a1], latest_euler[a2]
                    ang = angle_between_eulers(e1, e2)
                    dif = per_axis_diffs(e1, e2)
                    #n1 = clients[a1].device.name or a1 if a1 in clients else a1
                    #n2 = clients[a2].device.name or a2 if a2 in clients else a2
                    n1 = a1
                    n2 = a2
                    if ang is not None:
                        print(f"{n1} ↔ {n2}: Total={ang:.1f}° | "
                              f"Heading={dif['heading']:.1f}° | "
                              f"Pitch={dif['pitch']:.1f}° | "
                              f"Roll={dif['roll']:.1f}°")
            print("====================================")
    return handler

# ====== CONNECT / SCAN ======
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
    return [d for d in found if SERVICE_UUID in {u.lower() for u in (d.metadata.get("uuids") or [])}]

# ====== MAIN ======
async def main():
    print("Starting (gyro + Euler only, no checksum)…")

    # Discover & connect
    devices = await scan_targets()
    if not devices:
        print("⚠️ No devices found. Make sure your Nicla(s) are advertising.")
    else:
        await asyncio.gather(*[connect_device(d) for d in devices])

    # Run until Ctrl-C
    loop = asyncio.get_event_loop()
    stop_future = loop.create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_future.set_result, None)

    await stop_future

    # Cleanup
    print("Shutting down…")
    for c in list(clients.values()):
        try:
            await c.disconnect()
        except Exception:
            pass
    print("Done.")

if __name__ == "__main__":
    asyncio.run(main())
