import asyncio
from collections import deque
from dataclasses import fields
import os
import struct
from time import time
from bleak import BleakScanner, BleakClient
from bleak.uuids import normalize_uuid_str
from bleak.backends.device import BLEDevice
from bleak.backends.characteristic import BleakGATTCharacteristic
import h5py
import numpy as np

from ..hermes.aidwear.utils.types import NiclaData, ExoNiclaMapping

service_uuid = "19b10000-e8f2-537e-4f6c-d104768a1214"
char_uuid = "19b10002-e8f2-537e-4f6c-d104768a1214"

mac_mapping = {
    "torso": "CF:16:02:45:A6:3B",
    "thigh_right": "74:50:7F:A5:C2:18",
    "thigh_left": "45:7B:E2:8D:B1:F6",
    "shank_right": "27:6D:73:51:55:D9",
    "shank_left": "BA:FE:71:7C:DA:D8",
}

device_mapping = ExoNiclaMapping(
    **dict(zip(mac_mapping.keys(), mac_mapping.keys()))
)  # validates input mapping.
device_data: dict[str, deque[NiclaData]] = dict(
    map(lambda field: (field.name, deque()), fields(device_mapping))
)
offsets: dict[str, float] = dict(
    map(lambda field: (field.name, 0.0), fields(device_mapping))
)
offsets_lock = asyncio.Lock()

discovered_devices: dict[str, BLEDevice] = {}
connected_devices: dict[str, BleakClient] = {}
disconnected_devices: dict[str, tuple[BleakClient, float]] = {}
service_uuid = normalize_uuid_str(service_uuid)
char_uuid = normalize_uuid_str(char_uuid)


async def cleanup() -> None:
    print("Cleaning up Niclas.", flush=True)
    await asyncio.gather(
        *(c.stop_notify(char_uuid) for c in connected_devices.values())
    )
    await asyncio.gather(*(c.disconnect() for c in connected_devices.values()))


async def discover() -> bool:
    discovered_devices = await BleakScanner.discover(
        timeout=5.0, service_uuids=[service_uuid]
    )
    found = list(map(lambda device: device.address, discovered_devices))
    if not all([(mac in found) for mac in mac_mapping.values()]):
        not_found = [name for name, mac in mac_mapping.items() if mac not in found]
        print(
            f"Couldn't find {not_found}.\n",
            "Make sure all Niclas are advertising.",
            flush=True,
        )
        return False

    inverted_mac_mapping = {v: k for k, v in mac_mapping.items()}
    discovered_devices = {
        inverted_mac_mapping[device.address]: device
        for device in filter(
            lambda d: d.address in mac_mapping.values(), discovered_devices
        )
    }
    return True


async def connect_all() -> bool:
    try:
        async with asyncio.TaskGroup() as tg:
            client_tasks = [
                tg.create_task(connect_and_subscribe(name, device))
                for name, device in discovered_devices.items()
            ]
        return all([t.result() for t in client_tasks])
    except Exception as e:
        print("Failed to connect to some of the Niclas.\n", e, flush=True)
        return False


async def connect_and_subscribe(name: str, device: BLEDevice) -> bool:
    client = BleakClient(
        device, disconnected_callback=make_disconnection_callback(name, device)
    )
    try:
        await client.connect()
        print(f"Connected to {name} [{device.address}]", flush=True)
        await client.start_notify(char_uuid, make_data_callback(name))
        connected_devices[name] = client
        return True
    except Exception as e:
        print(f"Failed to connect to {name}: {e}", flush=True)
        return False


async def reconnect_manager(is_cleanup_event):
    while not is_cleanup_event.is_set():
        for name, (device, _) in list(disconnected_devices.items()):
            print(f"Trying to reconnect to {name}...", flush=True)

            fresh_device = await BleakScanner.find_device_by_address(
                device.address, timeout=1.0
            )
            if not fresh_device:
                print(f"Device {name} not found in scan.", flush=True)
                continue

            success = await connect_and_subscribe(name, fresh_device)
            if success:
                print(f"Reconnected to {name}.", flush=True)
                disconnected_devices.pop(name, None)
            else:
                print(f"Reconnect to {name} failed, will retry later.", flush=True)
        await asyncio.sleep(1.5)


async def wait_for_input(is_cleanup_event):
    loop = asyncio.get_event_loop()
    while not is_cleanup_event.is_set():
        result = await loop.run_in_executor(None, input, "Press 'Q' to exit...")
        if result == "Q":
            is_cleanup_event.set()


def make_disconnection_callback(name: str, device: BLEDevice):
    def callback(client: BleakClient) -> None:
        print(f"Device {name} [{device.address}] disconnected.", flush=True)
        connected_devices.pop(name, None)
        disconnected_devices[name] = (device, time())

    return callback


def make_data_callback(name: str):
    def callback(characteristic: BleakGATTCharacteristic, raw_data: bytearray) -> None:
        sample = NiclaData.from_bytes(raw_data)
        device_data[name].append(sample)

    return callback


def write_hdf5():
    filename_hdf5 = "nicla.hdf5"
    filepath_hdf5 = os.path.join(".", filename_hdf5)
    num_to_append = 0
    while os.path.exists(filepath_hdf5):
        num_to_append += 1
        filename_hdf5 = "%s_%02d.hdf5" % ("nicla", num_to_append)
        filepath_hdf5 = os.path.join(".", filename_hdf5)

    with h5py.File(filepath_hdf5, "w") as hdf5_file:
        for device_name, data in device_data.items():
            device_group = hdf5_file.create_group(device_name)

            gyro = (el.gyroscope for el in data)
            roll = (el.euler for el in data)
            timestamp = (el.toa_s for el in data)

            device_group.create_dataset(
                name="gyro",
                data=np.array(gyro),
                maxshape=(None,),
                dtype="f32",
                chunks=True,
            )
            device_group.create_dataset(
                name="euler",
                data=np.array(roll),
                maxshape=(None,),
                dtype="f32",
                chunks=True,
            )
            device_group.create_dataset(
                name="time_ms",
                data=np.array(timestamp),
                maxshape=(None,),
                dtype="f32",
                chunks=True,
            )
    return


async def main() -> None:
    # 1) Discover IMUs.
    while not (result := await discover()):
        print("Trying to rediscover Niclas. Make sure all are on.", flush=True)
        await asyncio.sleep(2)

    # 2) Connect all BLE IMUs.
    while not (result := await connect_all()):
        await cleanup()
        print("Trying to reconnect to Niclas.", flush=True)
        await asyncio.sleep(2)

    is_cleanup_event = asyncio.Event()
    # 3) Main working loop.
    await asyncio.gather(
        reconnect_manager(is_cleanup_event),
        wait_for_input(is_cleanup_event),
    )

    # Save `device_data`.
    write_hdf5()

    # 4) Cleanup on exit.
    await cleanup()


if __name__ == "__main__":
    asyncio.run(main())
