# Nicla Sense ME BLE Optimization & Tuning Guide

This document outlines the operating system settings, Bluetooth Low Energy (BLE) protocol parameters, and architectural optimizations required to achieve stable, high-rate streaming with multiple Nicla Sense ME sensors connected to a Raspberry Pi central.

---

## 1. Multi-Peripheral BLE Bottlenecks

Streaming concurrently from **5 Nicla Sense ME sensors** over BLE to a single Raspberry Pi central encounters physical and OS-level constraints:

1. **RF Time-Division Multiplexing (TDMA):** A single Bluetooth transceiver must interleave connection events across 5 separate peripherals. If connection intervals are wide (e.g., 30–50 ms), the physical link cannot sustain $>30$–$40\text{ Hz}$ per sensor.
2. **Disconnection Cascade via Active Scanning:** If one sensor drops and the central initiates an active LE scan (`BleakScanner`), the radio stops servicing active connections on other channels, starving remaining sensors and causing all 5 to disconnect.
3. **Firmware Buffer Overflows:** Producing samples at 100 Hz while the BLE link drains at 40 Hz overflows the peripheral's internal TX ring buffer, resulting in connection timeouts.
4. **D-Bus & Serialization Overhead:** 5 sensors streaming at 100 Hz produces 500 individual GATT notifications and D-Bus signals per second, placing high CPU load on Python and the system D-Bus daemon.

---

## 2. Raspberry Pi OS Kernel & BlueZ Configuration

When peripherals request tighter connection parameters, the Linux kernel Bluetooth subsystem (`bluetooth.ko`) will clamp or reject the request unless its allowed bounds are adjusted.

### A. Kernel Connection Parameters (Immediate Test)
Run the following commands on the Raspberry Pi:

```bash
# Set allowed connection interval bounds in 1.25 ms units (8 = 10 ms, 16 = 20 ms)
sudo sh -c 'echo 8 > /sys/kernel/debug/bluetooth/hci0/conn_min_interval'
sudo sh -c 'echo 16 > /sys/kernel/debug/bluetooth/hci0/conn_max_interval'

# Set supervision timeout in 10 ms units (300 = 3000 ms)
sudo sh -c 'echo 300 > /sys/kernel/debug/bluetooth/hci0/supervision_timeout'
```

Verify the active settings:
```bash
cat /sys/kernel/debug/bluetooth/hci0/conn_min_interval   # prints 8
cat /sys/kernel/debug/bluetooth/hci0/conn_max_interval   # prints 16
cat /sys/kernel/debug/bluetooth/hci0/supervision_timeout # prints 300
```

### B. Persistent Kernel Configuration (Across Reboots)
Create a systemd `tmpfiles.d` configuration file:

```bash
sudo tee /etc/tmpfiles.d/bluetooth-le.conf << 'EOF'
w /sys/kernel/debug/bluetooth/hci0/conn_min_interval - - - - 8
w /sys/kernel/debug/bluetooth/hci0/conn_max_interval - - - - 16
w /sys/kernel/debug/bluetooth/hci0/supervision_timeout - - - - 300
EOF
```

### C. BlueZ Configuration (`/etc/bluetooth/main.conf`)
Ensure BlueZ allows low connection intervals for LE peripherals:

```bash
sudo nano /etc/bluetooth/main.conf
```

Add or verify the following under the `[LE]` section:
```ini
[LE]
MinConnectionInterval=8
MaxConnectionInterval=16
ConnectionSupervisionTimeout=300
```

Restart the Bluetooth service:
```bash
sudo systemctl restart bluetooth
```

### D. 2.4 GHz Wi-Fi / Bluetooth Radio Coexistence
The Raspberry Pi 5 uses a combined Cypress/Broadcom wireless chip sharing a single antenna for both 2.4 GHz Wi-Fi and Bluetooth.
* Heavy 2.4 GHz Wi-Fi activity causes radio contention and packet drops for BLE.
* **Recommendation:** Connect the Raspberry Pi 5 via **5 GHz Wi-Fi** or **Ethernet**.
* If 2.4 GHz Wi-Fi must be used, disable power saving to prevent periodic scan/sleep latency spikes:
  ```bash
  sudo iwconfig wlan0 power off
  ```

### E. D-Bus Performance Tuning (`dbus-broker`)
At high notification rates (e.g., 500 notifications/sec), Debian's default `dbus-daemon` can consume significant CPU time on a single core. Switching to `dbus-broker` provides 3–5x lower IPC latency and reduced CPU overhead:

```bash
sudo apt install -y dbus-broker
sudo systemctl enable --now dbus-broker.service
```

---

## 3. Firmware & Backend Design Patterns

### A. Non-Blocking Event Handlers
Avoid any blocking loops (`delay()`) inside `ArduinoBLE` event callbacks (`connectDataHandler` / `disconnectDataHandler`). Blocking in callbacks halts Mbed OS BLE polling, causing GATT service discovery timeouts on the central.

### B. Connection Parameter Requests
In the Arduino firmware `setup()`:
```cpp
// Set connection interval: 8 to 16 (10 ms to 20 ms)
BLE.setConnectionInterval(8, 16);
// Set supervision timeout: 300 (3.0 seconds)
BLE.setSupervisionTimeout(300);
```

### C. Sensor Output Data Rate (ODR) & Zero Latency
Configure virtual sensors with explicit rates and 0 ms report latency:
```cpp
acc.begin(100.0f, 0);
gyr.begin(100.0f, 0);
euler.begin(100.0f, 0);
```
* **Latency $= 0$:** Ensures the Bosch BHI260AP sensor fusion hub streams samples immediately to its hardware FIFO without power-saving batching delays.

### D. Avoid Active Scanning During Active Streaming
In the Python backend (`NiclaBleBackend.run()`):
* **Never call `BleakScanner.find_device_by_address` while other devices are streaming.**
* Connect directly using `BleakClient(device, ...)`. Linux BlueZ creates a direct LE connection (`HCI_LE_Create_Connection`) without commanding the radio into active channel-hopping scan mode.

### E. Sequential Connection Establishment
Connect peripherals sequentially with a 200–300 ms settle delay rather than using `asyncio.gather(...)`. Bluetooth controllers only support one pending connection creation procedure at a time at the HCI layer.

---

## 4. Advanced Architecture: Measurement Batching

When streaming from 5 sensors, the primary physical bottleneck is **packet count over the air**, not bandwidth.

### Comparison: 1-Sample vs. 4-Sample Batching

| Metric | 1 Sample / Packet (Current) | 4 Samples / Packet (Batched) |
| :--- | :--- | :--- |
| **Physical Sampling Rate** | 100 Hz (1 sample / 10 ms) | **100 Hz (1 sample / 10 ms)** |
| **Transmissions per Sensor** | 100 packets/sec | **25 packets/sec** |
| **Total Air Packets (5 Sensors)** | **500 packets/sec** | **125 packets/sec** ($-75\%$) |
| **D-Bus Signal Events** | **500 events/sec** | **125 events/sec** ($-75\%$) |
| **Python Queue Pushes** | **500 calls/sec** | **125 calls/sec** ($-75\%$) |
| **Transmission Latency** | $\sim 10\text{ ms}$ | $\sim 30\text{–}40\text{ ms}$ buffer window |

### Packet Layout for Batching
Instead of sending 33 bytes every 10 ms:

1. **Packet Header (10 bytes):**
   * Modality Mask (`uint8_t`) — 1 byte
   * Base Timestamp (`uint32_t`) — 4 bytes
   * Base Sequence ID (`uint32_t`) — 4 bytes
   * Batch Sample Count $N$ (`uint8_t`) — 1 byte
2. **Payload ($N \times 24$ bytes):**
   * Sample 0: Acc (6B) + Gyro (6B) + Euler (12B)
   * Sample 1: Acc (6B) + Gyro (6B) + Euler (12B)
   * Sample 2: Acc (6B) + Gyro (6B) + Euler (12B)
   * Sample 3: Acc (6B) + Gyro (6B) + Euler (12B)
3. **Total Packet Size:** $\approx 106\text{ bytes}$ (well within BLE 4.2 / 5.0 ATT MTU of 247 bytes).

### Latency vs. Throughput Recommendations
* **For High-Rate Closed-Loop Reflex Control ($<20\text{ ms}$ window):** Use $N = 2$ batching (50 pkts/s per sensor, 250 total pkts/s, 10–20 ms latency).
* **For Gait Phase Estimation, Logging, & AI Trajectory Models:** Use $N = 4$ batching (25 pkts/s per sensor, 125 total pkts/s, 30–40 ms latency).
