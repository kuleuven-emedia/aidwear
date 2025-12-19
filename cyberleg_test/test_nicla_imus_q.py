import asyncio
import struct
import signal
from bleak import BleakScanner, BleakClient
from bleak.backends.device import BLEDevice

SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214".lower()
CHAR_UUID = "19b10002-e8f2-537e-4f6c-d104768a1214".lower()


def make_callback(name):
    def callback(_, raw_data: bytearray):
        ts, w, x, y, z = struct.unpack("<Iffff", raw_data)
        print(f"[{name}] {ts:6d} ms | w={w:.4f}, x={x:.4f}, y={y:.4f}, z={z:.4f}")

    return callback


async def connect_and_subscribe(device: BLEDevice) -> tuple[str, BleakClient]:
    name = device.name or device.address
    client = BleakClient(device)
    await client.connect()
    await client.start_notify(CHAR_UUID, make_callback(name))
    print(f"â Connected to {name} [{device.address}]")
    return device.address, client


async def main():
    # 1) Discover
    print("🔍 Scanning for NiclaQuat devices…")
    found = await BleakScanner.discover(return_adv=True, timeout=5.0)
    targets = [
        device
        for device, adv_data in found.values()
        if SERVICE_UUID in {uuid.lower() for uuid in adv_data.service_uuids}
    ]
    if not targets:
        print("⚠️  No devices found. Make sure both Niclas are advertising.")
        return

    # 2) Connect to each in parallel
    try:
        async with asyncio.TaskGroup() as tg:
            client_tasks = [tg.create_task(connect_and_subscribe(d)) for d in targets]
    except ExceptionGroup as e:
        print("â ï¸  Failed to connect to some of the Niclas.\n", e)
        return

    clients = dict([t.result() for t in client_tasks])

    # 3) Keep running until Ctrl-C
    loop = asyncio.get_event_loop()
    stop_future = loop.create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_future.set_result, None)
    await stop_future

    # 4) Clean up
    print("🔌 Disconnecting…")
    await asyncio.gather(*(c.disconnect() for c in clients.values()))


if __name__ == "__main__":
    asyncio.run(main())
