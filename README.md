# 🦾 3-Axis Robotic Arm Controller & Simulator

A high-performance, precision-controlled 3-axis robotic arm project featuring a mathematical Inverse Kinematics (IK) engine, a robust Arduino-based servo controller with EEPROM state persistence, and interactive command-line and graphical dashboards.


> 🎬 **Demo** — ![Watch smooth_glider.py in action](media/demo_smooth_glider.gif)

---


## 🌟 Key Features

* **Precision Inverse Kinematics**: Custom analytical geometric IK engine solving joint coordinates in real time for precise target tracking.
* **Sub-Degree Resolution Serial Protocol**: A binary serial packet protocol scaling float angles to centidegrees (1/100th of a degree) and packing them into compact 16-bit big-endian structures for optimal transmission speed and high accuracy.
* **Non-Volatile Memory (EEPROM)**: The Arduino firmware caches successful servo coordinates to the EEPROM on every valid command, preventing joint snapping/jerking when powered off and rebooted.
* **Startup Position Handshake**: On boot, the Arduino reads its last-known position from EEPROM (clamping uninitialized states to a safe $90^\circ$), moves there, and outputs an `INIT:base,bottom,top,gripper` handshake signal to coordinate seamless starting glides.
* **Multi-Mode Trajectory Planners**: Supports both immediate step-coordinate jumps (`rigid_glider.py`) and joint-space linear angle interpolation (`smooth_glider.py`) for smooth path planning.
* **3D Printable Mechanical Assets**: CAD models (STL/3MF/F3Z formats) located in the workspace under `version_1/` for physical arm manufacturing and prototyping.

---

# Assembly

## 🛠️ Hardware Requirements

1. **Microcontroller**: Arduino Uno, Nano, or similar AVR/ARM board.
2. **Servos**: 4x standard micro-servos (e.g., SG90 or MG90S).
   - **Servo 1**: Turntable Base (Pin 3)
   - **Servo 2**: Shoulder / Lower Arm (Pin 5)
   - **Servo 3**: Elbow / Upper Arm (Pin 6)
   - **Servo 4**: Gripper Claw (Pin 9)
3. **Power**: 5V external power supply (recommended for servos to prevent USB current limit trip).
4. **Screws**: 
    - 8x **M2 x 10mm**
    - 4x **M2.5 x 5mm**

## 🖨️ 3D Printing the parts

Located under **version_1** in the **final_print_parts** folder is the full final .stl files for printing, the settings used for a Bambu Labs P2S were the generic print settings with supports on. 


## 🪛 Assembly

1. After 3D printing the parts place the first servo into the base

![Step 1 - Servo seated in base housing](media/construction_step_1.png)

2. Then screw the top of the base onto the the servo

![Step 2 - Base top placed onto servo](media/construction_step_2.png)

3. Now screw the lower arm and servo together

![Step 3 - Lower arm assembled with servo](media/construction_step_3.png)

4. Slot the lower arm into the base 

![Step 4 - Lower arm slotted into base](media/construction_step_4.png)

5. Insert the servo to the end of the lower arm and upper arm and screw the upper arm to the lower arm 

![Step 5 - Upper arm connected at elbow joint](media/construction_step_5.png)

6. Screw the gripper onto the end of the upper arm

![Step 6 - Gripper attached, full assembly complete](media/construction_step_6.png)
---

## 🚀 Getting The Arm Moving

### 1. Arduino Setup
1. Open [robot_arm_controller.ino](robot_arm_controller/robot_arm_firmware/robot_arm_controller.ino) in the **Arduino IDE**.
2. Connect your Arduino board via USB.
3. Select your Board and Port from the Tools menu.
4. Click **Upload** to flash the firmware.

### 2. Python Environment Setup
Install the necessary Python dependencies for the UI and serial driver:
```bash
pip install pygame pyserial pandas
```

### 3. Running the Controllers

#### 🎮 Interactive CLI (TUI) Modes:
* **Cartesian Control (X, Y, Z, Gripper)**:
  Allows typing target millimeter coordinates directly into the console.
  ```bash
  python robot_arm_controller/point_tui.py
  ```
* **Raw Angle Control (Base, Bottom, Top, Gripper)**:
  Allows writing raw joint-space angles.
  ```bash
  python robot_arm_controller/angle_tui.py
  ```

#### 📈 Waypoint Macro Executions:

* **Smooth Glider (Interpolated)**:
  Prompts for your waypoint CSV path, handshakes startup coordinates from the Arduino, glides smoothly to the starting point, and interpolates through each subsequent waypoint segment.
  ```bash
  python robot_arm_controller/smooth_glider.py
  ```
* **Rigid Glider (Immediate Steps)**:
  Prompts for a CSV path and executes sequential coordinates by sleeping for the designated duration between movements.
  ```bash
  python robot_arm_controller/rigid_glider.py
  ```

* **CSV formating**:

  | **x cordinate** (mm), **y cordinate** (mm), **z cordinate** (mm), **gripper angle** (degree), **time** (seconds)
---

## 📐 Serial Communication Protocol

To ensure sub-degree accuracy without transmission overhead, the system uses a custom binary protocol using big-endian. Angles are scaled to **centidegrees** (multiplied by 100), converted to signed 16-bit integers, and sent as an 8-byte packet:

| Byte Index | Data Packed | Description |
|---|---|---|
| **0** | `Base High Byte` | Most Significant Byte of Base Angle |
| **1** | `Base Low Byte` | Least Significant Byte of Base Angle |
| **2** | `Shoulder High Byte` | Most Significant Byte of Shoulder/Bottom Angle |
| **3** | `Shoulder Low Byte` | Least Significant Byte of Shoulder/Bottom Angle |
| **4** | `Elbow High Byte` | Most Significant Byte of Elbow/Top Angle |
| **5** | `Elbow Low Byte` | Least Significant Byte of Elbow/Top Angle |
| **6** | `Gripper High Byte` | Most Significant Byte of Gripper Angle |
| **7** | `Gripper Low Byte` | Least Significant Byte of Gripper Angle |

---

## ⚙️ Mathematical Specification

The inverse kinematics algorithm maps $3\text{D}$ space $(X, Y, Z)$ into cylindrical coordinates, using `atan2` for robust base rotation, and solves planar arm equations via the Law of Cosines:

$$d = \sqrt{x_{inline}^2 + z_{inline}^2}$$

$$\theta_2 = \arccos\left(\frac{L_1^2 + L_2^2 - d^2}{2 L_1 L_2}\right)$$
