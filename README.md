# 🦾 3-Axis Robotic Arm Controller & Simulator

A high-performance, precision-controlled 3-axis robotic arm project featuring a mathematical Inverse Kinematics (IK) engine, a robust Arduino-based servo controller, and an interactive, real-time Pygame graphical dashboard.

---

## 🌟 Key Features

* **📐 Precision Inverse Kinematics**: Custom analytical geometric IK engine solving joint coordinates in real time for precise target tracking.
* **🔌 Sub-Degree Resolution Serial Protocol**: A binary serial packet protocol scaling float angles to centidegrees (1/100th of a degree) and packing them into compact 16-bit big-endian structures for optimal transmission speed and high accuracy.
* **🛠️ 3D Printable Mechanical Assets**: Production-ready, 3D-printable industrial arm models (STL/3MF format) located in the workspace for easy prototyping and manufacturing.
* **📟 Interactive CLI (`main.py`)**: A lightweight command-line controller for quick diagnostics and manual coordinate entry.

---

## 📂 Project Directory Structure

```filepath
├── README.md                        # Project documentation
├── version_1/                       # 3D printable CAD models for the arm
│   ├── 1.1
│   ├── 1.2                          # First printed version
│   ├── 1.3                          # Fixes to the lower arm and upper base
│   ├── 1.4                          # Latest additions to the arm
│ 
│
└── Robot_Arm_Controller/            # Hardware control & simulation scripts
    ├── main.py                      # Interactive console CLI controller
    ├── num_packer.py                # Serial data compression helper
    ├── kinematics.py                # Mathematical IK solvers
    ├── go_to.py                     # Sends the angles to the arduino
    ├── arm_controller_gui.py        # ⚠️ In development ⚠️ GUI for controlling the robotic arm
    │
    └── robot_arm_controller/        # Arduino firmware
        ├── robot_arm_controller.ino # Main serial packet loop
        └── servo_controller.ino     # Microsecond servo driver
```

---

## 🛠️ Hardware Requirements

1. **Microcontroller**: Arduino Uno, Nano, or any AVR/ARM board.
2. **Servos**: 4x standard micro-servos (e.g., SG90 or MG90S).
   - **Servo 1**: Turntable Base (Pin 3)
   - **Servo 2**: Shoulder / Lower Arm (Pin 5)
   - **Servo 3**: Elbow / Upper Arm (Pin 6)
   - **Servo 4**: Gripper Claw (Pin 9)
3. **Power**: 5V external power supply (recommended for servos to prevent USB current limit trip).
4. **Screws**: 
    - 8x **M2 x 10mm**
    - 4x **M2.5 x 5mm**

---

## 🚀 Getting Started

### 1. Arduino Setup
1. Open the [robot_arm_controller.ino](file:///Users/tomilan/projects/robotic_arm/Robot_Arm_Controller/robot_arm_controller/robot_arm_controller.ino) in the **Arduino IDE**.
2. Connect your Arduino board via USB.
3. Select your Board and Port from the Tools menu.
4. Click **Upload** to flash the firmware.

### 2. Python Environment Setup
Install the necessary Python dependencies for the UI and serial driver:
```bash
pip install pygame pyserial
```

### 3. Run the Controllers
Configure your USB Modem port at the top of the python scripts, then run either the CLI or the GUI:

* **CLI Controller**:
  ```bash
  python Robot_Arm_Controller/main.py
  ```
* **GUI Controller (In Dev)**:
  ```bash
  python Robot_Arm_Controller/arm_controller_gui.py
  ```

---

## 📐 Serial Communication Protocol

To ensure sub-degree accuracy without transmission overhead, the system uses a custom binary protocol. Angles are scaled to **centidegrees** (multiplied by 100), converted to signed 16-bit integers, and sent as a 6-byte packet:

| Byte Index | Data Packed | Description |
|---|---|---|
| **0** | `Base High Byte` | Most Significant Byte of Base Angle |
| **1** | `Base Low Byte` | Least Significant Byte of Base Angle |
| **2** | `Shoulder High Byte` | Most Significant Byte of Shoulder Angle |
| **3** | `Shoulder Low Byte` | Least Significant Byte of Shoulder Angle |
| **4** | `Elbow High Byte` | Most Significant Byte of Elbow Angle |
| **5** | `Elbow Low Byte` | Least Significant Byte of Elbow Angle |

---

## ⚙️ Mathematical Specification

The inverse kinematics algorithm maps $3\text{D}$ space $(X, Y, Z)$ into cylindrical coordinates, using `atan2` for robust base rotation, and solves planar arm equations via the Law of Cosines:

$$d = \sqrt{x_{inline}^2 + z_{inline}^2}$$

$$\theta_2 = \arccos\left(\frac{L_1^2 + L_2^2 - d^2}{2 L_1 L_2}\right)$$

---
