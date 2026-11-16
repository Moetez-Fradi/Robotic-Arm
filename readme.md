# 6-Axis Color-Sorting Robotic Arm (Distributed CAN Architecture)

Control firmware and Host PC Python stack for a 6-DOF color-sorting manipulator powered by 4 distributed `STM32F103` CAN joint nodes (`RoboticArmPCB` Rev E) and an `ESP32` USB-to-CAN bridge.

## Repository Structure

```text
├── firmware/
│   ├── esp32_can_bridge/
│   │   └── esp32_can_bridge.ino   # Standalone USB (921600 baud) <-> CAN (500 kbps) bridge
│   └── stm32_joint_node/
│       └── joint_node_fw.ino      # Standalone firmware for all 4 STM32 joint PCBs
├── vision/
│   ├── README.md                  # Camera setup, calibration, usage, coordinate frame
│   ├── detector.py                # One-frame cube colour + position detection (OpenCV)
│   ├── mat.py                     # Printable pick mat (ArUco markers + grey colour reference)
│   ├── calibrate.py               # Lens and colour calibration tool
│   ├── live.py                    # `Vision` class for the motion code + live viewer
│   └── docs/                      # Simulated test report, design book section 6
└── software/
    ├── requirements.txt           # Python dependencies
    ├── arm_interface.py           # Low-level USB-CAN driver & 50 Hz telemetry listener
    ├── sim_virtual_arm.py         # Digital twin simulator (ESP32 + 4x STM32 nodes)
    ├── verify_subsystem.py        # Automated 5-stage subsystem verification suite
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

## Computer Vision

The `vision/` package tells the motion code **whether there is a cube, its colour (`red` / `blue`) and its position** in the arm's frame. It uses one USB webcam looking at a printed pick mat (ArUco markers + a grey colour reference). It needs only `opencv-python` and `numpy`. Run the commands from the repository root:

```bash
pip install -r vision/requirements.txt
python -m vision.mat                                           # makes the A4 pick mat to print
python -m vision.calibrate colours --camera 0 --names red blue # store the two cube colours
python -m vision.live --camera 0                               # live viewer
```

```python
from vision.live import Vision

r = Vision(camera=0).read_stable()   # r.colour, r.confidence, r.x_mm, r.y_mm, r.z_mm (arm frame, mm)
```

Tested on simulated camera frames only (100 % correct colour over 600 frames with varied lighting); the real webcam is not tested yet. See [`vision/README.md`](vision/README.md) for setup, calibration and the coordinate frame the motion code should expect.

## Flashing Hardware

* **ESP32 Bridge:** Flash `esp32_can_bridge.ino` (`ESP32 Dev Module`, `921600` baud). Wire `GPIO 5` (`TX`) and `GPIO 4` (`RX`) to the `SN65HVD230` transceiver with a `120 Ω` termination resistor across `CAN_H` / `CAN_L`.
* **STM32 Joint Nodes (×4):** Flash `joint_node_fw.ino` (`Generic STM32F1 series` → `BluePill F103C8`, **USB support: `None`** so `PA11`/`PA12` remain dedicated to `CAN1`). Set node IDs via `JP1–JP3` (`000` to `011`) and close `JP4` (`120 Ω` termination) on Node 4.
