#!/usr/bin/env python3
import os
import can
import struct
import time

from hermes.aidwear.prosthesis.utils.types import EncoderId
from hermes.aidwear.prosthesis.utils.utils import config_can_linux
from hermes.utils.time_utils import get_time_str, get_time


def log(msg):
    ts = get_time_str(get_time(), format="%H:%M:%S.%f")[:-3]
    print(f"[{ts}] {msg}")


def listen_can():
    bus = can.interface.Bus(channel="can0", interface="socketcan", fd=True)  # bustype
    print("Reading CAN data bus...\n")
    try:
        while True:
            msg = bus.recv(1.0)
            if msg is None:
                continue

            data = msg.data
            id = msg.arbitration_id

            if id in [EncoderId.KNEE.value, EncoderId.ANKLE.value]:
                if len(data) >= 2:
                    angle_raw = (data[0] << 8) | data[1]

                    error = (angle_raw >> 14) & 0x01  # verification error flag (bit 14)

                    angle_data = angle_raw & 0x3FFF  # 14 data bits
                    angle_deg = angle_data * (
                        360.0 / 16384.0
                    )  # Resolution of the 14 bit => 2^14 = 16384

                    log(
                        f"[{EncoderId(id).name}] AS5048A SPI -> Raw = {bin(angle_raw)[2:].zfill(16)} | Angle = {angle_deg:06.2f}deg | Steps = {angle_data:05}"
                    )
                else:
                    log(
                        f"[{EncoderId(id).name}] AS5048A SPI -> Data too short | Raw = {data.hex()}"
                    )

            else:
                log(
                    f"[{hex(id)}] unknowned bus -> raw data = {data.hex()} (DLC={len(data)})"
                )

    except KeyboardInterrupt:
        print("\n interruption asked by the user")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    config_can_linux()
    time.sleep(0.5)
    listen_can()
