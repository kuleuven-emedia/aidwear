# AidWear
Prosthesis controller, wrapped with KU Leuven's [HERMES](https://github.com/maximyudayev/hermes) framework to communicate to an external AI controller for intent-based locomotion mode selection and fatigue-based support level moderation.

The prosthesis uses a hierarchical controller, with each layer controlling the layer below it:
- High-level -> AI-based intent (ambulation mode) and fatigue forecasting
- Mid-level -> intra-mode normative kinematics trajectory tracking
- Low-level -> field-oriented motor control

<p align="center">
  <img src="images/overview.png" alt="Overview of continuous operation of the prosthesis" />
</p>

These mid-level state machines are defined based on the derived locomotion kinematic equations for the specific rigid body, within the degrees-of-freedom it offers, to track a desired normative motion trajectory with a desired frequency response of the control system.

## Installation
### General devices
Use [UV](https://docs.astral.sh/uv/getting-started/installation/#installing-uv) Python package and project manager to create, activate, and configure the environment on the corresponding device:

| Device | Command |
| - | - |
| Prosthesis - RPi 5 | `uv sync --extra prosthesis` |
| AI device - test laptop / RPi 5 | `uv sync --extra ai` |

### Jetson Orin NX
We are using a Jetson Orin NX 16GB on a [Seeed Studio A608 carrier board](https://www.seeedstudio.com/Jetson-A608-Carrier-Board-for-Orin-NX-Orin-Nano-Series-p-5853.html).

#### Prerequisites
* JetPack 6.2
* CUDA 12.6
* LT 36.4.3
* Python 3.10.12
* FFmpeg 8.1 ([Jetson patch](https://github.com/Keylost/jetson-ffmpeg))

Follow the Seeed Studio [instructions](https://wiki.seeedstudio.com/reComputer_A608_Flash_System/) to flash the board.

> [!IMPORTANT]
> Update the contents of the tools shell scripts to set a custom username and password, and to activate VNC. Take note of the IP addresses for the IPoverUSB and the Eth interfaces for easy connection w/o a display.

> [!IMPORTANT]
> Use a fan on the Jetson, or it will thermal reset.

> [!IMPORTANT]
> Remove the jumper after flashing the board to boot into the OS.

> [!WARNING]
> After flashing the Jetson, installing some of the following packages or running may not work due to the lack of some NVIDIA libraries. Install a matching JetPack SDK or simply drop the error trace into Gemini to help find NVIDIA Jetson forums that list the name of the shared libraries to install with `apt-get`. 

#### Step-by-step
1. Follow the instructions to compile FFmpeg (8.1) from source with the [Jetson-specific patch](https://github.com/Keylost/jetson-ffmpeg) for hardware-accelerated video handling.
1. Clone the AidWear project:
   ```bash
   git clone git@github.com:kuleuven-emedia/aidwear.git
   ```
1. Create a virtual environment:
   ```bash
   python -m venv .venv
   ```
1. Install Numpy 1.26.4 to build UVC wheel for Pupil Labs Core smartglasses:
   ```bash
   source .venv/bin/activate && pip install numpy==1.26.4 && cd ..
   ```
1. Follow the instructions to compile UVC for the [Pupil Labs Core smartglasses](https://github.com/pupil-labs/pyuvc#install-from-source) and to run it as non-user.
1. Install the Jetson-optimized PyTorch toolchains:
   ```bash
   cd ./aidwear && pip install -r jetson_requirements.txt
   ```
1. Setup Chrony NTP client (`/etc/chrony/chrony.conf`) to prefer to poll the local network NTP server with `minpoll` and `maxpoll` of 4, like: `server 10.220.25.99 iburst minpoll 4 maxpoll 4 prefer`.
   
   Set the public servers to less frequent polling to not abuse them and to not get blacklisted: `pool ntp.ubuntu.com iburst maxsources 4 minpoll 10 maxpoll 15`.
1. Setup a DHCP server on the `enP1p1s0` ethernet interface for 10.220.24.1/24.
1. Install any extra needed Python packages using only pip in that virtual environment.
1. Optimize RAM on the Jetson and use an NVMe swap, following [these instructions](https://www.jetson-ai-lab.com/tutorials/ram-optimization/).

## Running
From the project root directory run the relevant convenience launcher:
| Scenario | Device | Command |
| - | - | - |
| Standalone prosthesis w/ manual CLI control | Prosthesis | `. run/prosthesis_standalone_cli/prosthesis.sh` |
| Standalone prosthesis w/ manual GUI app control | Prosthesis | `. run/prosthesis_standalone_gui/prosthesis.sh` |
| Full-fledged sensing setup w/ HERMES | Experiment orchestrator (e.g. laptop or NUC) | `. run/ai_<model_type>/master.bat` |

This will automatically wrap the corresponding YAML configurations into the rest of the HERMES execution environment to seemlessly interconnect different sensing, processing, and actuating components. It will also update experiment metadata to create a unique data folder with recorded files.

> [!IMPORTANT]
> The dual-controller setup automatically spawns the HERMES orchestrator between the AI device and the prosthesis embedded controller. Even better, it lets you launch distributed sensing setup from your laptop and have remote shell terminals pop-up to interact with each HERMES-enabled device throughout the experiment.

> [!WARNING]
> To stop the prosthesis for safety reasons, without killing the experiment, enter 'S' in the terminal of the experiment orchestrator (e.g. researcher's laptop that spawned remote terminal shell for prosthesis control - applicable to any mode of prosthesis operation), or the VNC remote desktop session, for HERMES-orchestrated experiments and local on-device running, respectively. 

> [!NOTE]
> (Discouraged) You can also generically run the HERMES CLI to manually configure in-line any desired arguments `hermes-cli -o ./data -f run/prosthesis_standalone_cli/prosthesis.yml -e project=<PROJECT> trial=<X>`
> Make sure to update the `trial` number on every launch of the script. It's used to create unique folders for data collection that avoid overwriting previously collected data. HERMES system will not allow you to run the same experiment name twice, to protect the previously collected data.

## IMU sensor placement
<p align="center">
  <img src="images/xsens_axes.jpg" alt="Axes orientation overview on Xsens IMUs" width="45%" />
  <img src="images/nicla_axes.png" alt="Axes orientaiton overview on Nicla Sense ME IMUs" width="45%" />
</p>

> [!IMPORTANT]
> AidWear placed Xsens IMUs with LED up, frontally on mid-thigh, mid-foot, pelvis, and laterally above ankle. Integrated Nicla IMUs are all placed frontally. (1) Correct and consistent orientation must be ensured, (2) correct axes mapping from Nicla to Xsens must be done to match expected AI model inputs. Currently, handled by the [`IntentClassifierPipeline`](/src/hermes/aidwear/ai_intent/pipeline.py).

AidWear axes matching:
| Location | Xsens | Nicla |
| - | - | - |
| Pelvis | [x,y,z] | [-y,x,z] |
| Thigh right | [x,y,z] | [-y,z,-x] |
| Thigh left | [x,y,z] | [-y,-z,x] |
| Shank right | [x,y,z] | [-y,z,-x] |
| Shank left | [x,y,z] | [-y,-z,x] |
| Foot right | [x,y,z] | [-y,x,z] |
| Foot left | [x,y,z] | [-y,x,z] |

### Manual standalone operation
When running in [standalone CLI mode](#option-1-local-shell-terminal), press the activity id on the keyboard, followed by 'Enter' to manually switch the prosthesis controller to it:
| Activity | ID |
| - | - |
| Idle | 0 |
| Walking | 1 |
| Sit-To-Stand | 2 |
| Stair Ascent | 3 |
| Stair Descent | 4 |

And enter a percentage of fatigue to manually update the level of assistance of the prosthesis controller by '%', followed by number 0-100, followed by 'Enter' (e.g. `$> %70`).

## Structure
The prosthesis-specific files:
```
src/
└── hermes/
    └── aidwear/
        ├── cli/
        │   ├── producer.py
        │   └── data_container.py
        ├── gui/
        │   ├── utils/
        │   │   └── types.py
        │   ├── producer.py
        │   └── data_container.py
        ├── ai_intent/
        │   ├── utils/
        │   │   ├── configs/
        │   │   ├── models/
        │   │   ├── weights/
        │   │   ├── config.py
        │   │   ├── datastructures.py
        │   │   ├── handler.py
        │   │   ├── transforms.py
        │   │   ├── types.py
        │   │   └── utils.py
        │   ├── data_container.py
        │   └── pipeline.py
        ├── ai_fatigue/
        │   ├── data_container.py
        │   ├── pipeline.py
        │   └── utils/
        └── prosthesis/
            ├── controller/
            │   ├── mode_selection.py
            │   └── prosthesis_handler.py
            ├── state_machines/
            │   ├── base.py
            │   ├── hurdle.py
            │   ├── idle.py
            │   ├── sit_to_stand.py
            │   ├── stair_ascent.py
            │   ├── stair_descent.py
            │   └── walking.py
            ├── can_control/
            │   ├── emulator_epos.py
            │   ├── motor_epos.py
            │   └── pmu_mateksys.py
            ├── sensors/
            │   ├── nicla/
            │   │   ├── abstract_backend.py
            │   │   ├── ble_backend.py
            │   │   └── i2c_backend.py
            │   └── can_backend.py
            ├── utils/
            │   ├── config_manager.py
            │   ├── types.py
            │   └── utils.py
            ├── data_container.py
            └── pipeline.py
```
### Prosthesis
 - `prosthesis_handler.py` is the main AsyncIO routine that manages the functionality of the prosthesis:
    1. Manages a persistent BLE/I2C connection to the onboard Nicla Sense ME motion sensors;
    1. Manages a persistent CAN connection to the onboard motors;
    1. Reads incoming IMU, motors, and telemetry data;
    1. Indicates upstream AI/UI intent transitions to `mode_selection` to trigger locomotion mode change;
    1. Continuously passes data to the currently active locomotion mode state machine to control the prosthesis;
 - `mode_selection.py` contains the macro state machine logic that manages safe transitions between the different locomotion modes of the prosthesis, based on the upstream high-level AI/UI controller.
 - `state_machines/` folder contains all state machines logic for each locomotion mode FSM (states, transitions, gait phase estimation, impedance/torque control commands to the motors, etc. - including `idle`, `walking`, `sit_to_stand`, `stair_ascent`, `stair_descent`, and `hurdle`).
 - `types.py` defines convenience datatypes that ensure safe typing between different coupled components of the system.
 - `utils.py` shared static logic, reusable by multiple distinct components.

#### Inputs
 - `can_backend.py` decouples CAN motor/PMU data receiving and parsing logic for both, virtual (testing) and physical (prosthesis) CAN setups.
 - `nicla/<ble/i2c>_backend.py` decouples specified Nicla Sense ME physical connection (BLE or I2C) with a uniform plug-and-play interface.
 - `nicla/types.py` defines convenience datatypes for the Nicla Sense ME IMU sensors to reuse throughout the system.

#### Outputs
 - `motor_epos.py` complete Python `ctypes` wrapper and communication primitives for Maxon EPOS4 motor controllers.
 - `pmu_mateksys.py` placeholder to control the MatekSys DroneCAN power monitor unit.
 - `emulator_epos.py` contains a callable meant to be run in a thread/process to create a virtual CAN bus that generates simulated motor data for validation of the overall system logic.

> [!TIP]
> **Motor Control Selection Guide:**  
> When choosing motor control primitives for wearable robotics or humanoid joints, consult the **[EPOS4 Firmware Specification & Robotics Use Case Guide](/docs/epos4_firmware_guide.md)**.
> It provides an **operating mode decision tree** (comparing Current/Torque Mode vs. CSP vs. IPM vs. PPM vs. Homing) along with natural-language use-case examples grounded in physical human-robot interaction (such as virtual impedance control during stance, sensorless hard-stop zeroing, and coordinated swing-phase trajectory tracking).
> For technical C-signatures and `ctypes` bindings of all 201 library functions, see the **[EPOS Command Library Reference](/docs/epos_command_library.md)**.

> [!TIP]
> The state chart controlling the prosthesis (i.e. `mode_selection.py` and state machines inside `state_machines/`) are built on [**python-statemachine**](https://python-statemachine.readthedocs.io/en/latest/index.html) package and can be visualized using the [built-in graph generator](https://python-statemachine.readthedocs.io/en/latest/diagram.html).

### HERMES
#### Prosthesis
 - `prosthesis/pipeline.py` HERMES Node integrating the prosthesis with the rest of the sensing and companion computing ecosystem.
 - `prosthesis/data_container.py` HERMES data container containing all the data generated by the prosthesis.

#### (Option #1) Local shell terminal
 - `cli/producer.py` HERMES Node enabling CLI based manual selection of intent [0, 4] and fatigue level [%0, %100] (must start from %, will autoclip values to 0-100 range), when using the prosthesis in standalone mode.
 - `cli/data_container.py` HERMES data container containing all valid CLI entries of intent and fatigue.

#### (Option #2) Remote phone GUI
 - `gui/producer.py` Android smartphone app enabling manual selection of intent [0, 4] and fatigue level [0, 100] (will autoclip values to 0-100 range), when using the prosthesis with a user-friendly smartphone interface.
 - `gui/data_container.py` HERMES data container containing all valid GUI entries of intent and fatigue.

#### (Option #3) Companion AI device
 - `ai_intent/pipeline.py` & `ai_fatigue/pipeline.py` PyTorch models enabling automated selection of intent [0, 4] and fatigue level [0, 100], when using the prosthesis in an end-to-end automated approach.
 - `ai_intent/data_container.py` & `ai_fatigue/data_container.py` HERMES data containers containing all valid AI predictions of intent and fatigue.
 - `ai_intent/utils/` Folder contains all the multimodal AI constructors and helping utilities to compose PyTorch processing pipelines for live inference.


### Nicla Sense ME
The [`prosthesis.yml`](/run/prosthesis_standalone_cli/prosthesis.yml#L50) file offers an option to configure the system to use [BLE or I2C](/run/prosthesis_standalone_cli/prosthesis.yml#L50) for communication with the onboard [Nicla motion sensors](https://docs.arduino.cc/hardware/nicla-sense-me/). The `device_mapping` must be correspondingly selected (commented/uncommented) and updated with the correct MAC addresses (in the case of BLE).

Use [PlatformIO](https://platformio.org/) to program and debug the firmware as needed, instead of the limited Arduino IDE.

#### (Option #1) - `BLE`
Uses the `bleak` package on the Raspberry Pi to receive wireless Bluetooth Low Energy data packets from the Nicla Sense ME devices. Quality is subject to RF interference from other devices, throughput limitation, BLE issues, lack of continuous synchronization between devices.

The prosthesis uses the integrated motion sensors in a "Push" strategy, where the sensors push the latest motion information into the prosthesis, to drive its internal logic. The prosthesis uses the most up-to-date and lowest latency kinematics knowledge, without any attempts at synchronizing data. This allows it to maintain tight soft realtime guarantees.

The [BLE firmware](/sensors_firmware/nicla_ble/nicla_ble.ino) compiles with flags that enable desired modalities - by default, gyroscope and Euler orientation data.

The sensors visualize the state of the sensors with the onboard LED for easier troubleshooting and validation of the health of the prosthesis.

![LED blink pattern for different states of the BLE motion sensors](/images/nicla_leds.gif)

**Battery and Power Management**
The battery-powered Nicla firmware supports battery monitoring and remote shutdown:
1. **Power On:** Press the Nicla's reset button to wake it up.
2. **Read Battery:** During operation, connect using the nRF Connect app (or similar) to monitor the battery percentage under the standard Battery Service (UUID `180F`, auto-detected).
3. **Shutdown (Ship Mode):** At the end of use, send a Write command of `0x01` to the custom BLE characteristic (UUID `1002`). This completely shuts down the Nicla's power management IC until the reset button is pressed again.
> [!IMPORTANT]
> Current gyroscope + euler configuration (with 9 bytes of metadata) is 27 total bytes/packet, practically limited to 40Hz for 5 concurrently streaming wireless sensors without batching and OS tuning.
> For Raspberry Pi OS kernel settings, BlueZ parameter adjustments, disconnection prevention, and measurement batching architecture, see the [Nicla BLE Optimization & Tuning Guide](/docs/nicla_ble_tuning.md).

#### (Option #2) - `I2C`
The integrated motion and environment sensors are configured to use the [TWIS1](https://docs-be.nordicsemi.com/bundle/nRF52832_PS_v1.9/raw/resource/enus/nRF52832_PS_v1.9.pdf#%5B%7B%22num%22%3A776%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C56.692%2C752.879%2Cnull%5D) peripheral for I2C communication, and exposed via the ESLOV connector.

Communicates to each Nicla Sense ME over a shared I2C bus on the [ESLOV connector (p.4)](https://docs.arduino.cc/resources/pinouts/ABX00050-full-pinout.pdf), via the Raspberry Pi's [`busio`](https://docs.circuitpython.org/en/latest/shared-bindings/busio) package.
To avoid polluting the I2C bus and wasting Pi's resources, each Nicla indicates via an interrupt that new data is available to be read.
The Raspberry Pi then accesses the corresponding device by its I2C address to retrieve the new (burst) data.

> [!IMPORTANT]
> Make sure to wire the INT pin (`P0_19`) of each of the Nicla's 5-pin ESLOV connectors to the Raspberry Pi's digital GPIO's [31](https://pinout.xyz/pinout/pin31_gpio6/), [11](https://pinout.xyz/pinout/pin11_gpio17/), [13](https://pinout.xyz/pinout/pin13_gpio27/), [15](https://pinout.xyz/pinout/pin15_gpio22/), and [16](https://pinout.xyz/pinout/pin16_gpio23/), in any order, but consistent with the specification in the [`prosthesis.yml`](run/prosthesis_standalone_cli/prosthesis.yml) file. This does not conflict with the pin occupation of the attached [CAN-FD HAT](https://www.waveshare.com/wiki/2-CH_CAN_FD_HAT#Interfaces). E.g.:
> | Nicla | Raspberry Pi 5 pin | I2C address |
> | - | - | - |
> | `torso` | [GPIO6 (pin 31)](https://pinout.xyz/pinout/pin31_gpio6/) | 0x11 |
> | `thigh_right` | [GPIO17 (pin 11)](https://pinout.xyz/pinout/pin11_gpio17/) | 0x12 |
> | `shank_right` | [GPIO27 (pin 13)](https://pinout.xyz/pinout/pin13_gpio27/) | 0x13 |
> | `thigh_left` | [GPIO22 (pin 15)](https://pinout.xyz/pinout/pin15_gpio22/) | 0x14 |
> | `shank_left` | [GPIO23 (pin 16)](https://pinout.xyz/pinout/pin16_gpio23/) | 0x15 |

> [!IMPORTANT]
> Also make sure to flash each Nicla with the right firmware and uniquely addressable via the [`I2C_ADDRESS`](/sensors_firmware/nicla_i2c/nicla_i2c.ino#L5) macro, consistent with the address specified in the `prosthesis.yml` file.

The standard I2C interface on the Raspberry Pi 5 does not support clock stretching. Nicla's non-deterministic threaded processing does not timely detect the incoming I2C request and doesn't acknowledge transactions. Enable an alternative high-speed I2C interface on the Pi under by adding into the boot configuration file `/boot/firmware/config.txt`, and wiring SDA to [GPIO4 (pin 7)](https://pinout.xyz/pinout/pin7_gpio4/) and SCL to [GPIO5 (pin 29)](https://pinout.xyz/pinout/pin29_gpio5/), both of which conveniently do not interfere with the CAN-FD hat. No external pull-up resistors are needed.
```
dtoverlay=i2c2-pi5,pins_4_5,baudrate=400000
```

Restart and check if the kernel module is enabled with:
```bash
ls /dev/i2c-2
```

Install support tools for CLI-based verification. Will help detect if slave devices are not properly connected.
```bash
sudo apt-get install i2c-tools
```

Program all the Nicla devices in PlatformIO or Arduino IDE.
Run the convenient utility to detect if all devices are connected to the bus and are detected by the Pi:
```bash
i2cdetect -y 2
```

> [!WARNING]
> Verify signal integrity on the I2C bus. The wire harness packs SCL and SDA unshielded wires tightly together and at high clock speeds (400kHz) may cause cross-talk.

The integrated motion and environment sensors are configured to use the [TWIS1](https://docs-be.nordicsemi.com/bundle/nRF52832_PS_v1.9/raw/resource/enus/nRF52832_PS_v1.9.pdf#%5B%7B%22num%22%3A776%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C56.692%2C752.879%2Cnull%5D) peripheral for I2C communication, and exposed via the ESLOV connector. 

> [!IMPORTANT]
> The internal processing of the Nicla's RTOS stretches the I2C clock for ~500us per transaction. Consider batching multiple queued up samples into a single transaction to mask latency with throughput. This implies updating firmware and balancing the requested sample rate with the number of connected devices.

#### Updating firmware

Use [PlatformIO](https://platformio.org/) to program and debug the firmware as needed:
1. Install the PlatformIO VSCode extension.
1. Open an existing project inside PlatformIO dashboard, by opening the folder containin the `.ini` file (e.g. `sensors_firmware/nicla_ble`). This will automatically resolve any build dependencies.
1. Connect the device over micro-USB.
1. Change any desired macros in the main C file, to configure the firmware to produce desired modalities, at desired rate, and for battery-powered operation.
1. Press "Program" to build and flash the firmware.
1. Save the full MAC address of the device by scanning BLE devices with [nRF Connect](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile) smartphone app, and using the addresses to fill in Nicla MACs in the YAML config files.

## Networking
| device | eth0 | wlan0 |
| - | - | - |
| AI | 10.220.24.101/24 | 10.220.25.101/24 |
| Prosthesis | 10.220.24.102/24 | 10.220.25.102/24 | 
| Phone GUI | - | 10.220.25.105/24 |

## Testing (Linux / Windows)
For benchtop testing of the code (state machines, AI, new motion sensors firmware, etc.) CAN communication to/from motors is simulated by `emulator_epos` via the [`is_emulate_can`](/run/prosthesis_standalone_cli/prosthesis.yml#L71) flag in the configuration [`prosthesis.yml`](/run/prosthesis_standalone_cli/prosthesis.yml) file. The emulator is automatically launched by `prosthesis_handler` as a subprocess, once the flag is set to `True`. It creates a virtual CAN network using the [`virtualcan`](https://github.com/windelbouwman/virtualcan/tree/master) package.

Install the `virtualcan` package in a folder outside the current project:
```bash
git clone https://github.com/windelbouwman/virtualcan.git <path_to_new_virtualcan_folder>
```

Install first the general Python CAN package, then the cloned virtualcan package into the current virtual environment:
```bash
uv pip install <path_to_new_virtualcan_folder>/python
```

Install [Cargo and Rust](https://doc.rust-lang.org/cargo/getting-started/installation.html) to locally host a CAN simulation server (mandatory):
```bash
curl https://sh.rustup.rs -sSf | sh
```

Launch a virtualcan server in a separate terminal window, to enable simulated inter-process CAN communication between `prosthesis_handler` and `emulator_epos`:
```bash
cd <path_to_new_virtualcan_folder>/rust/server
cargo run --release -- --port 18881
```

Run the launch script the same way as for actual operation, from VSCode or via the bash script.
```bash
. run/prosthesis_standalone_cli/prosthesis.sh
```
