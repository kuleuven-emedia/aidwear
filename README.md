# AidWear
Cyberleg transfemoral prosthesis controller, wrapped with KU Leuven's [HERMES](https://github.com/maximyudayev/hermes) framework to communicate to an external AI controller for intent-based locomotion mode selection and fatigue-based support level moderation.

## Roadmap
- [ ] Implement motor communication to the Epos 4 Compact motors
- [ ] Implement state machines for each locomotion mode
- [ ] Choose the gain constants for the impedance controller

## Structure
The exoskeleton-specific files:
```
└──hermes/
   ...
   └──aidwear/
      ├──fsm/
      │  ├──cyberleg_handler.py
      │  ├──mode_selection.py
      │  └──state_machines.py
      ├──motor_control/
      │  ├──cubemars_can_commands.py
      │  ├──impedance_tuning.py
      │  └──torque_characterization.py
      ├──utils/
      │  ├──types.py
      │  └──utilities.py
      ...
```
 - `cyberleg_handler.py` is the main AsyncIO routine that manages the functionality of the prosthesis. It manages a persistent BLE connection to onboard Nicla Sense ME sensors, reads incoming IMU data from them, triggers transitions between locomotion mode state machines, and continuously passes data to the currently active state machine to control the prosthesis.
 - `mode_selection.py` contains the macro state machine logic that manages safe transitions between the different locomotion modes of the prosthesis, based on the upstream high-level controller.
 - `state_machines.py` contains all state machines logic for each locomotion mode FSM (states, transitions, actions, ...).
 - `cubemars_can_commands.py` contains all functions required to control the motors via CAN.
 - `types.py` defines convenience datatypes that ensure safe typing between different coupled components of the system.
 - `utilities.py` shared static logic, reusable by multiple distinct components.

## Installation
Create, activate, and configure a Python environment:
```bash
python -m venv .venv
.venv/bin/activate
pip install -r requirements.txt
```

## Networking
| device | eth0 | wlan0 |
| - | - | - |
| Prosthesis | 192.168.2.102/24 | 192.168.1.196/24 | 
| AI | 192.168.2.101/24 | NA |

## Running
From the project root directory run the convenience launcher:
```bash
. run.sh
```

This will wrap your custom `prosthesis.yml` configuration into the rest of the HERMES execution environment to seemlessly interconnect different sensing, processing, and actuating components. It will also update experiment metadata to create a unique data folder with recorded files.

> [!IMPORTANT]
> You can also generically run the HERMES CLI to manually configure in-line any desired arguments `hermes-cli -o ./data -f prosthesis.yml -e project=AidWear trial=<X>`
> Make sure to update the `trial` number on every launch of the script. It's used to create unique folders for data collection, to avoid overwriting previously collected data.
