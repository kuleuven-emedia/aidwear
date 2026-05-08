import asyncio
import struct
import signal
import time
import logging
import json
import csv
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError


# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.FileHandler("nicla_imu.log"), logging.StreamHandler()],
)
logger = logging.getLogger("NiclaIMU")

# Service and characteristic UUIDs
SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214".lower()
CHAR_UUID_QUAT = "19b10002-e8f2-537e-4f6c-d104768a1214".lower()
CHAR_UUID_GYRO = "19b10003-e8f2-537e-4f6c-d104768a1214".lower()
CHAR_UUID_EUL = "19b10004-e8f2-537e-4f6c-d104768a1214".lower()
CHAR_UUID_STATUS = "19b10005-e8f2-537e-4f6c-d104768a1214".lower()

# Global state
clients = {}  # address → client
device_stats = {}  # address → device stats
disconnected_devices = {}  # address → (device, disconnect time)
_connecting = set()  # addresses being connected
latest_orientations = {}  # address → (heading, pitch, roll)
relative_angles = {}  # (addr1, addr2) → {angle, heading_diff, pitch_diff, roll_diff}

# Create data directory for logging
data_dir = Path("imu_data")
data_dir.mkdir(exist_ok=True)


class DeviceStats:
    def __init__(self, addr, name):
        self.addr = addr
        self.name = name
        self.timestamp = time.strftime("%Y%m%d_%H%M%S")

        # Stats for each data type
        self.stats = {
            "quat": {"received": 0, "missed": 0, "last_seq": None, "last_ts": 0},
            "gyro": {"received": 0, "missed": 0, "last_seq": None, "last_ts": 0},
            "euler": {"received": 0, "missed": 0, "last_seq": None, "last_ts": 0},
        }

        # Connection stats
        self.connect_time = time.time()
        self.reconnect_count = 0
        self.disconnects = 0

        # Open data files for logging
        self.files = {
            "quat": open(
                data_dir / f"{name}_{addr}_quat_{self.timestamp}.csv", "w", newline=""
            ),
            "gyro": open(
                data_dir / f"{name}_{addr}_gyro_{self.timestamp}.csv", "w", newline=""
            ),
            "euler": open(
                data_dir / f"{name}_{addr}_euler_{self.timestamp}.csv", "w", newline=""
            ),
        }

        # Write headers
        csv.writer(self.files["quat"]).writerow(
            [
                "system_time",
                "device_time",
                "sequence",
                "w",
                "x",
                "y",
                "z",
                "checksum_valid",
            ]
        )
        csv.writer(self.files["gyro"]).writerow(
            ["system_time", "device_time", "sequence", "x", "y", "z", "checksum_valid"]
        )
        csv.writer(self.files["euler"]).writerow(
            [
                "system_time",
                "device_time",
                "sequence",
                "heading",
                "pitch",
                "roll",
                "checksum_valid",
            ]
        )

    def close(self):
        """Close all files"""
        for file in self.files.values():
            file.close()

    def calculate_efficiency(self, data_type):
        """Calculate reception efficiency for a data type"""
        stats = self.stats[data_type]
        total = stats["received"] + stats["missed"]
        if total == 0:
            return 100.0
        return 100.0 * stats["received"] / total

    def to_dict(self):
        """Convert stats to a dictionary for serialization"""
        return {
            "name": self.name,
            "address": self.addr,
            "connect_time": self.connect_time,
            "reconnect_count": self.reconnect_count,
            "disconnects": self.disconnects,
            "uptime": time.time() - self.connect_time,
            "data": {
                data_type: {
                    "received": stats["received"],
                    "missed": stats["missed"],
                    "efficiency": self.calculate_efficiency(data_type),
                }
                for data_type, stats in self.stats.items()
            },
        }


def verify_checksum(data, expected_checksum):
    """Verify the checksum of received data"""
    calculated = 0
    for b in data[:-1]:  # Skip the last byte which is the checksum
        calculated ^= b
    return calculated == expected_checksum


def compute_angle_between_imus(euler1, euler2):
    """
    Compute the angle between two IMUs using their Euler angles.

    Args:
        euler1: Tuple of (heading, pitch, roll) in degrees for first IMU
        euler2: Tuple of (heading, pitch, roll) in degrees for second IMU

    Returns:
        Angle in degrees between the two orientations
    """
    try:
        # Convert Euler angles to rotation matrices
        # ZYX order is typically used (heading=yaw, pitch, roll)
        r1 = Rotation.from_euler("zyx", euler1, degrees=True)
        r2 = Rotation.from_euler("zyx", euler2, degrees=True)

        # Compute relative rotation
        r_diff = r1.inv() * r2

        # Convert to angle-axis representation
        angle_axis = r_diff.as_rotvec()

        # Calculate magnitude of rotation
        angle_degrees = np.linalg.norm(angle_axis) * 180.0 / np.pi

        return angle_degrees
    except Exception as e:
        logger.error(f"Error computing angle between IMUs: {e}")
        return None


def analyze_relative_motion(euler1, euler2):
    """
    Analyze the relative motion between two IMUs in more detail.

    Returns a dict with angular differences in each axis.
    """
    heading1, pitch1, roll1 = euler1
    heading2, pitch2, roll2 = euler2

    # Calculate the smallest angle difference for each component
    # (accounting for wrap-around at 360 degrees)
    def angle_diff(a, b):
        diff = (a - b) % 360
        if diff > 180:
            diff = 360 - diff
        return diff

    return {
        "heading_diff": angle_diff(heading1, heading2),
        "pitch_diff": angle_diff(pitch1, pitch2),
        "roll_diff": angle_diff(roll1, roll2),
    }


def calculate_joint_angle(proximal_imu, distal_imu, joint_axis="y"):
    """
    Calculate anatomical joint angle between two IMUs.

    Args:
        proximal_imu: (heading, pitch, roll) of the proximal segment
        distal_imu: (heading, pitch, roll) of the distal segment
        joint_axis: The primary axis of rotation for this joint ('x', 'y', or 'z')

    Returns:
        Joint angle in degrees
    """
    try:
        # Convert Euler angles to rotation matrices
        r_proximal = Rotation.from_euler("zyx", proximal_imu, degrees=True)
        r_distal = Rotation.from_euler("zyx", distal_imu, degrees=True)

        # Calculate relative orientation
        r_relative = r_proximal.inv() * r_distal

        # Extract Euler angles from the relative orientation
        # We use different decomposition orders depending on the joint
        if joint_axis == "x":
            euler_order = "xyz"  # Rotation primarily in x-axis
        elif joint_axis == "y":
            euler_order = "yxz"  # Rotation primarily in y-axis
        else:
            euler_order = "zxy"  # Rotation primarily in z-axis

        angles = r_relative.as_euler(euler_order, degrees=True)

        # Return the angle corresponding to the primary axis
        primary_index = euler_order.index(joint_axis)
        return angles[primary_index]
    except Exception as e:
        logger.error(f"Error calculating joint angle: {e}")
        return None


def make_quat_handler(name, addr):
    """Notification handlers for different data types."""
    if addr not in device_stats:
        device_stats[addr] = DeviceStats(addr, name)

    stats = device_stats[addr]

    def handler(_, data: bytearray):
        try:
            system_time = time.time()
            sequence, timestamp, w, x, y, z, checksum = struct.unpack("<IIffffB", data)

            # Verify checksum
            checksum_valid = verify_checksum(data, checksum)
            if not checksum_valid:
                logger.warning(f"[{name}] QUAT checksum mismatch - Seq:{sequence}")

            # Check for missed packets
            if stats.stats["quat"]["last_seq"] is not None:
                expected_seq = stats.stats["quat"]["last_seq"] + 1
                if sequence > expected_seq:
                    missed = sequence - expected_seq
                    stats.stats["quat"]["missed"] += missed
                    if missed > 1:  # Only log if more than one packet missed
                        logger.warning(
                            f"[{name}] Missed {missed} QUAT packets ({expected_seq}..{sequence - 1})"
                        )

            # Update stats and log data
            stats.stats["quat"]["received"] += 1
            stats.stats["quat"]["last_seq"] = sequence
            stats.stats["quat"]["last_ts"] = timestamp

            # Log to file
            csv.writer(stats.files["quat"]).writerow(
                [system_time, timestamp, sequence, w, x, y, z, int(checksum_valid)]
            )

            # Only print occasionally to avoid flooding the console
            if sequence % 20 == 0:
                efficiency = stats.calculate_efficiency("quat")
                logger.info(
                    f"[{name}] QUAT | Seq:{sequence:6d} Time:{timestamp:6d}ms | "
                    f"w={w:.4f}, x={x:.4f}, y={y:.4f}, z={z:.4f} | "
                    f"Eff:{efficiency:.1f}%"
                )

        except Exception as e:
            logger.error(f"Error in QUAT handler for {name}: {e}")

    return handler


def make_gyro_handler(name, addr):
    if addr not in device_stats:
        device_stats[addr] = DeviceStats(addr, name)

    stats = device_stats[addr]

    def handler(_, data: bytearray):
        try:
            system_time = time.time()
            sequence, timestamp, x, y, z, checksum = struct.unpack("<IIfffB", data)

            # Verify checksum
            checksum_valid = verify_checksum(data, checksum)
            if not checksum_valid:
                logger.warning(f"[{name}] GYRO checksum mismatch - Seq:{sequence}")

            # Check for missed packets
            if stats.stats["gyro"]["last_seq"] is not None:
                expected_seq = stats.stats["gyro"]["last_seq"] + 1
                if sequence > expected_seq:
                    missed = sequence - expected_seq
                    stats.stats["gyro"]["missed"] += missed
                    if missed > 1:
                        logger.warning(
                            f"[{name}] Missed {missed} GYRO packets ({expected_seq}..{sequence - 1})"
                        )

            # Update stats and log data
            stats.stats["gyro"]["received"] += 1
            stats.stats["gyro"]["last_seq"] = sequence
            stats.stats["gyro"]["last_ts"] = timestamp

            # Log to file
            csv.writer(stats.files["gyro"]).writerow(
                [system_time, timestamp, sequence, x, y, z, int(checksum_valid)]
            )

            # Only print occasionally
            if sequence % 30 == 0:  # Print less frequently than quaternion data
                efficiency = stats.calculate_efficiency("gyro")
                logger.info(
                    f"[{name}] GYRO | Seq:{sequence:6d} Time:{timestamp:6d}ms | "
                    f"x={x:.2f}°/s, y={y:.2f}°/s, z={z:.2f}°/s | "
                    f"Eff:{efficiency:.1f}%"
                )

        except Exception as e:
            logger.error(f"Error in GYRO handler for {name}: {e}")

    return handler


def make_euler_handler(name, addr):
    if addr not in device_stats:
        device_stats[addr] = DeviceStats(addr, name)

    stats = device_stats[addr]

    def handler(_, data: bytearray):
        try:
            system_time = time.time()
            sequence, timestamp, heading, pitch, roll, checksum = struct.unpack(
                "<IIfffB", data
            )

            # Verify checksum
            checksum_valid = verify_checksum(data, checksum)
            if not checksum_valid:
                logger.warning(f"[{name}] EULER checksum mismatch - Seq:{sequence}")

            # Check for missed packets
            if stats.stats["euler"]["last_seq"] is not None:
                expected_seq = stats.stats["euler"]["last_seq"] + 1
                if sequence > expected_seq:
                    missed = sequence - expected_seq
                    stats.stats["euler"]["missed"] += missed
                    if missed > 1:
                        logger.warning(
                            f"[{name}] Missed {missed} EULER packets ({expected_seq}..{sequence - 1})"
                        )

            # Update stats and log data
            stats.stats["euler"]["received"] += 1
            stats.stats["euler"]["last_seq"] = sequence
            stats.stats["euler"]["last_ts"] = timestamp

            # Log to file
            csv.writer(stats.files["euler"]).writerow(
                [
                    system_time,
                    timestamp,
                    sequence,
                    heading,
                    pitch,
                    roll,
                    int(checksum_valid),
                ]
            )

            # Store the latest orientation for angle calculations
            latest_orientations[addr] = (heading, pitch, roll)

            # Compute angles between this IMU and all others
            for other_addr, other_angles in latest_orientations.items():
                if other_addr != addr:
                    # Make sure we always use the same order for the pair key
                    pair_key = tuple(sorted([addr, other_addr]))

                    # Calculate overall angle
                    angle = compute_angle_between_imus(
                        (heading, pitch, roll), other_angles
                    )

                    # Calculate per-axis differences
                    axis_diffs = analyze_relative_motion(
                        (heading, pitch, roll), other_angles
                    )

                    # Store all results
                    relative_angles[pair_key] = {"angle": angle, **axis_diffs}

            # Only print occasionally
            if sequence % 40 == 0:  # Print even less frequently
                efficiency = stats.calculate_efficiency("euler")
                logger.info(
                    f"[{name}] EULER | Seq:{sequence:6d} Time:{timestamp:6d}ms | "
                    f"heading={heading:.1f}°, pitch={pitch:.1f}°, roll={roll:.1f}° | "
                    f"Eff:{efficiency:.1f}%"
                )

        except Exception as e:
            logger.error(f"Error in EULER handler for {name}: {e}")

    return handler


def make_status_handler(name, addr):
    if addr not in device_stats:
        device_stats[addr] = DeviceStats(addr, name)

    def handler(_, data: bytearray):
        try:
            (
                uptime,
                quat_fill,
                gyro_fill,
                euler_fill,
                quat_seq_msb,
                gyro_seq_msb,
                euler_seq_msb,
                *_,
            ) = struct.unpack("<IBBBBBBB", data[:8])

            # Only log occasionally to avoid flooding
            if time.time() % 10 < 0.5:  # Roughly every 10 seconds
                logger.info(
                    f"[{name}] STATUS | Uptime:{uptime / 1000:.1f}s | "
                    f"Buffer Fill: QUAT:{quat_fill}% GYRO:{gyro_fill}% EULER:{euler_fill}%"
                )
        except Exception as e:
            logger.error(f"Error in STATUS handler for {name}: {e}")

    return handler


async def connect_device(device, retry_count=0):
    """Connect to a device and subscribe to notifications"""
    addr = device.address
    name = device.name or addr

    # Skip if already connected or connecting
    if addr in clients and clients[addr].is_connected:
        return True
    if addr in _connecting:
        return False

    _connecting.add(addr)
    logger.info(f"Connecting to {name} [{addr}]...")

    try:
        # Create client with disconnect callback
        client = BleakClient(
            device, disconnected_callback=lambda c: handle_disconnect(device)
        )

        # Connect with timeout
        await client.connect(timeout=10.0)
        logger.info(f"✅ Connected to {name} [{addr}]")

        # Initialize stats if needed
        if addr not in device_stats:
            device_stats[addr] = DeviceStats(addr, name)
        else:
            device_stats[addr].reconnect_count += 1

        # Subscribe to all characteristics
        await client.start_notify(CHAR_UUID_QUAT, make_quat_handler(name, addr))
        await client.start_notify(CHAR_UUID_GYRO, make_gyro_handler(name, addr))
        await client.start_notify(CHAR_UUID_EUL, make_euler_handler(name, addr))
        await client.start_notify(CHAR_UUID_STATUS, make_status_handler(name, addr))

        # Store client reference
        clients[addr] = client

        return True

    except Exception as e:
        logger.error(f"❌ Failed to connect to {name} [{addr}]: {e}")

        # Retry logic
        if retry_count < 2:  # Try up to 3 times (0, 1, 2)
            logger.info(f"Retrying connection to {name} (attempt {retry_count + 2}/3)")
            await asyncio.sleep(1)  # Wait between retries
            _connecting.discard(addr)
            return await connect_device(device, retry_count + 1)

        return False

    finally:
        _connecting.discard(addr)


def handle_disconnect(device):
    """Handle device disconnection"""
    addr = device.address
    name = device.name or addr

    # Update stats
    if addr in device_stats:
        device_stats[addr].disconnects += 1

    logger.warning(f"⚠️ Device {name} [{addr}] disconnected!")

    # Remove from clients but keep device info for reconnection
    clients.pop(addr, None)

    # Schedule reconnection
    disconnected_devices[addr] = (device, time.time())


# Background tasks
async def reconnect_manager():
    """Periodically check for disconnected devices and attempt to reconnect"""
    while True:
        try:
            current_time = time.time()
            to_remove = []

            # Try to reconnect devices that have been disconnected for at least 5 seconds
            for addr, (device, disconnect_time) in disconnected_devices.items():
                if current_time - disconnect_time >= 5.0:
                    name = device.name or addr
                    logger.info(f"Attempting to reconnect to {name} [{addr}]...")

                    success = await connect_device(device)
                    if success:
                        to_remove.append(addr)
                    else:
                        # Update disconnect time for next attempt
                        disconnected_devices[addr] = (device, current_time)

            # Remove reconnected devices
            for addr in to_remove:
                disconnected_devices.pop(addr, None)

        except Exception as e:
            logger.error(f"Error in reconnect manager: {e}")

        # Check every 5 seconds
        await asyncio.sleep(5.0)


async def stats_manager():
    """Periodically save device statistics"""
    while True:
        try:
            # Save stats to JSON file
            stats_data = {addr: stats.to_dict() for addr, stats in device_stats.items()}

            with open(data_dir / "device_stats.json", "w") as f:
                json.dump(stats_data, f, indent=2)

            # Log a summary to console every minute
            if int(time.time()) % 60 == 0:
                logger.info("=== Device Statistics Summary ===")
                for addr, stats in device_stats.items():
                    logger.info(
                        f"{stats.name} [{addr}]: "
                        f"QUAT: {stats.calculate_efficiency('quat'):.1f}% | "
                        f"GYRO: {stats.calculate_efficiency('gyro'):.1f}% | "
                        f"EULER: {stats.calculate_efficiency('euler'):.1f}% | "
                        f"Reconnects: {stats.reconnect_count}"
                    )
                logger.info("================================")

        except Exception as e:
            logger.error(f"Error in stats manager: {e}")

        # Update every 10 seconds
        await asyncio.sleep(10.0)


async def angle_reporter():
    """Periodically report angles between IMUs"""
    # Initialize angles log file
    with open(data_dir / "relative_angles.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "timestamp",
                "device1",
                "device2",
                "angle",
                "heading_diff",
                "pitch_diff",
                "roll_diff",
            ]
        )

    while True:
        try:
            # Only report if we have at least 2 IMUs with orientation data
            if len(latest_orientations) >= 2:
                # Write to file
                with open(data_dir / "relative_angles.csv", "a", newline="") as f:
                    writer = csv.writer(f)
                    current_time = time.time()

                    for (addr1, addr2), angle_data in relative_angles.items():
                        name1 = (
                            device_stats[addr1].name if addr1 in device_stats else addr1
                        )
                        name2 = (
                            device_stats[addr2].name if addr2 in device_stats else addr2
                        )

                        # Write to CSV file
                        writer.writerow(
                            [
                                current_time,
                                addr1,
                                addr2,
                                angle_data["angle"],
                                angle_data["heading_diff"],
                                angle_data["pitch_diff"],
                                angle_data["roll_diff"],
                            ]
                        )

                # Report to console every 5 seconds
                if int(time.time()) % 5 == 0:
                    logger.info("=== Relative Angles Between IMUs ===")
                    for (addr1, addr2), angle_data in relative_angles.items():
                        name1 = (
                            device_stats[addr1].name if addr1 in device_stats else addr1
                        )
                        name2 = (
                            device_stats[addr2].name if addr2 in device_stats else addr2
                        )

                        logger.info(
                            f"{name1} ↔ {name2}: "
                            f"Total:{angle_data['angle']:.1f}° | "
                            f"Heading:{angle_data['heading_diff']:.1f}° | "
                            f"Pitch:{angle_data['pitch_diff']:.1f}° | "
                            f"Roll:{angle_data['roll_diff']:.1f}°"
                        )
                    logger.info("====================================")

        except Exception as e:
            logger.error(f"Error in angle reporter: {e}")

        # Update every second
        await asyncio.sleep(1.0)


async def scan_for_devices():
    """Scan for Nicla IMU devices"""
    logger.info("🔍 Scanning for Nicla IMU devices...")

    try:
        found = await BleakScanner.discover(timeout=5.0)
        targets = [
            d
            for d in found
            if SERVICE_UUID in {u.lower() for u in (d.metadata.get("uuids") or [])}
        ]

        if not targets:
            logger.warning("⚠️ No devices found. Make sure Niclas are advertising.")
            return []

        logger.info(
            f"Found {len(targets)} devices: {', '.join(d.name or d.address for d in targets)}"
        )
        return targets

    except Exception as e:
        logger.error(f"Error scanning for devices: {e}")
        return []


async def discovery_loop():
    """Periodically scan for new devices"""
    while True:
        try:
            # Find new devices
            devices = await scan_for_devices()

            # Connect to devices not already connected
            connect_tasks = []
            for device in devices:
                addr = device.address
                if addr not in clients and addr not in _connecting:
                    connect_tasks.append(connect_device(device))

            if connect_tasks:
                await asyncio.gather(*connect_tasks)

        except Exception as e:
            logger.error(f"Error in discovery loop: {e}")

        # Scan every 30 seconds
        await asyncio.sleep(30.0)


async def main():
    try:
        logger.info("Starting Nicla IMU data collection and angle analysis system...")

        # Initial device scan
        devices = await scan_for_devices()
        if not devices:
            logger.warning("No devices found. Will continue scanning...")

        # Connect to found devices
        await asyncio.gather(*[connect_device(d) for d in devices])

        # Start background tasks
        reconnect_task = asyncio.create_task(reconnect_manager())
        stats_task = asyncio.create_task(stats_manager())
        discovery_task = asyncio.create_task(discovery_loop())
        angles_task = asyncio.create_task(angle_reporter())

        # Run until interrupted
        loop = asyncio.get_event_loop()
        stop_future = loop.create_future()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, stop_future.set_result, None)

        logger.info("System running. Press Ctrl+C to stop.")
        await stop_future

        # Clean up background tasks
        logger.info("Shutting down...")
        reconnect_task.cancel()
        stats_task.cancel()
        discovery_task.cancel()
        angles_task.cancel()

        # Disconnect all clients
        logger.info("🔌 Disconnecting devices...")
        for addr, client in list(clients.items()):
            try:
                await client.disconnect()
            except:
                pass

        # Close all files
        for stats in device_stats.values():
            stats.close()

        # Save final statistics
        stats_data = {addr: stats.to_dict() for addr, stats in device_stats.items()}
        with open(data_dir / "final_stats.json", "w") as f:
            json.dump(stats_data, f, indent=2)

        # Save final angle data
        with open(data_dir / "final_angles.json", "w") as f:
            angle_data = {
                f"{addr1}_{addr2}": data
                for (addr1, addr2), data in relative_angles.items()
            }
            json.dump(angle_data, f, indent=2)

        logger.info("System shut down gracefully.")

    except Exception as e:
        logger.critical(f"Fatal error: {e}", exc_info=True)


if __name__ == "__main__":
    asyncio.run(main())
