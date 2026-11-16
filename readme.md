# 6-Axis Color-Sorting Robotic Arm (Distributed CAN Architecture)

Control firmware and Host PC Python stack for a 6-DOF color-sorting manipulator powered by 4 distributed `STM32F103` CAN joint nodes (`RoboticArmPCB` Rev E) and an `ESP32` USB-to-CAN bridge.

## Repository Structure

```text
├── firmware/
│   ├── esp32_can_bridge/
│   │   └── esp32_can_bridge.ino   # Standalone USB (921600 baud) <-> CAN (500 kbps) bridge
│   └── stm32_joint_node/
│       └── joint_node_fw.ino      # Standalone firmware for all 4 STM32 joint PCBs
└── software/
    ├── requirements.txt           # Python dependencies
    ├── arm_interface.py           # Low-level USB-CAN driver & 50 Hz telemetry listener
    ├── motion_api.py              # High-level IK & pick-and-place API (Vision handoff)
    ├── sim_virtual_arm.py         # Digital twin simulator (ESP32 + 4x STM32 nodes)
    ├── verify_subsystem.py        # Automated 5-stage subsystem verification suite
    └── manual_jog.py              # Interactive CLI for single-joint hardware bring-up
```

## Quick Start (Simulation Mode — No Hardware Required)

1. Install Python dependencies:
   ```bash
   cd software
   pip install -r requirements.txt
   ```
2. Start the virtual arm simulator in **Terminal 1**:
   ```bash
   python sim_virtual_arm.py
   ```
3. Run the automated verification suite in **Terminal 2**:
   ```bash
   python verify_subsystem.py
   ```

## Vision Pipeline Integration (`motion_api.py`)

Import `ArmMotionController` and call `sort_object(x_mm, y_mm, color)` with the detected object's workspace coordinates (in millimeters relative to the arm base) and color label (`"RED"`, `"GREEN"`, `"BLUE"`, `"YELLOW"`):

```python
from motion_api import ArmMotionController

# Use "socket://127.0.0.1:5555" for simulation, or "COM5" / "/dev/ttyUSB0" on hardware
arm_ctrl = ArmMotionController(port="socket://127.0.0.1:5555")
arm_ctrl.sort_object(x_mm=220.0, y_mm=40.0, color="RED")
```

## Flashing Hardware

* **ESP32 Bridge:** Flash `esp32_can_bridge.ino` (`ESP32 Dev Module`, `921600` baud). Wire `GPIO 5` (`TX`) and `GPIO 4` (`RX`) to the `SN65HVD230` transceiver with a `120 Ω` termination resistor across `CAN_H` / `CAN_L`.
* **STM32 Joint Nodes (×4):** Flash `joint_node_fw.ino` (`Generic STM32F1 series` → `BluePill F103C8`, **USB support: `None`** so `PA11`/`PA12` remain dedicated to `CAN1`). Set node IDs via `JP1–JP3` (`000` to `011`) and close `JP4` (`120 Ω` termination) on Node 4.