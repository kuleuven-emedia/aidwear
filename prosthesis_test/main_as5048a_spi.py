#!/usr/bin/env python3
import os
import can
import struct
import time
import csv

# --- Configure CAN FD with BRS ---
def setup_can_interface():
    print("Configuration de can0...")
    os.system("sudo ip link set can0 down")
    os.system("sudo ip link set can0 type can bitrate 500000 dbitrate 500000 fd on loopback off restart-ms 1000 berr-reporting on")
    os.system("sudo ip link set can0 up")
    print("CAN0 interface initialised.\n")


# --- Timestamp Function
def log(msg):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}")    

# --- Reading CAN data bus ---
def listen_can():
    bus = can.interface.Bus(channel='can0', interface='socketcan', fd=True) # bustype
    print("Reading CAN data bus...\n")
    try:
        while True:
            msg = bus.recv(1.0)
            if msg is None:
                continue

            data = msg.data
            id = msg.arbitration_id
            
            # # >>> DEBUG BRUT POUR VERIFIER LE FD <<<
            # print(f"id=0x{id:X} is_fd={msg.is_fd} len={len(data)} data={data.hex()}")

            # --- AS5048A SPI ---
            if id in (0x201,0x202):
                if len(data) >= 2:
                    angle_raw = (data[0] << 8) | data[1]
                    
                    # Vérification error flag (bit 14)
                    ef = (angle_raw >> 14) & 0x01

                    angle_data = angle_raw & 0x3FFF  # 14 bits utiles
                    angle_deg = angle_data * (360.0 / 16384.0) #resolution de 14 bit ici => 2^14 = 16384
                    
                    log(f"[{hex(id)}] AS5048A SPI -> Raw = {bin(angle_raw)[2:].zfill(16)} | Angle = {angle_deg:06.2f}deg | Steps = {angle_data:05}")
                else:
                    log(f"[{hex(id)}] AS5048A SPI -> Data too short | Raw = {data.hex()}")

            # --- Unknowned bus ---
            else:
                log(f"[0x{id:X}] unknowned bus -> raw data = {data.hex()} (DLC={len(data)})")

    except KeyboardInterrupt:
        print("\n interruption asked by the user")
    finally:
        os.system("sudo ip link set can0 down")
        print("CAN0 Interface desactivated.")

# --- Programme principal ---
if __name__ == "__main__":
    setup_can_interface()
    time.sleep(0.5)
    listen_can()
