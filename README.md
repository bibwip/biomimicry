# ROS 2 & Arduino Manipulator Control System

This project implements a teleoperation system for a mobile robot (e.g., Leo Rover) equipped with a custom 3-axis manipulator. It uses a PS4 Controller to simultaneously drive the robot via ROS 2 and control the manipulator via a Serial connection to an Arduino.

## 📂 File Overview

| File | Description |
| :--- | :--- |
| `controller_com.py` | **ROS 2 Node.** Reads PS4 inputs, publishes `cmd_vel` to the robot, and sends Serial packets to the Arduino. |
| `controller_receiver.ino` | **Arduino Firmware.** Receives Serial packets, calculates inverse kinematics, and drives Steppers/Servos. |
| `controller_tester.py` | **Diagnostic Script.** Tests the Controller-to-Arduino connection without needing ROS. |

---

## 🔌 Hardware Setup

### Wiring Diagram (Arduino)
Based on the macros defined in `controller_receiver.ino`:

| Component | Pin Type | Arduino Pin |
| :--- | :--- | :--- |
| **Stepper X** | Step / Dir | 2 / 5 |
| **Stepper Y** | Step / Dir | 3 / 6 |
| **Stepper Z** | Step / Dir | 4 / 7 |
| **Claw Servo** | PWM Signal | A0 |
| **Storage Servo** | PWM Signal | A1 |
| **PS4 Controller** | Bluetooth/USB | Connect to PC/Raspberry Pi |

> **Note:** Ensure your Stepper Drivers (A4988/DRV8825) and Servos are powered by an external power supply (12V/5V), not just the Arduino USB.

---

## 🎮 Controls Mapping

The system uses a **"Split Control"** scheme:
* **Left Stick:** Drives the Robot base (ROS 2).
* **Right Stick & Buttons:** Controls the Manipulator (Arduino).

| Input | Action | Logic |
| :--- | :--- | :--- |
| **Left Stick Y** | Robot Forward/Back | ROS `cmd_vel.linear.x` |
| **Left Stick X** | Robot Rotate Left/Right | ROS `cmd_vel.angular.z` |
| **Right Stick** | Move Manipulator Planar | Mapped to Stepper kinematic X/Y |
| **L1** | Close Claw | Servo moves + |
| **R1** | Open Claw | Servo moves - |
| **L2** | Arm Retract (Down) | Z-axis -1 |
| **R2** | Arm Extend (Up) | Z-axis +1 |
| **Circle (◯)** | Open Storage | Servo to 110° |
| **Triangle (△)** | Close Storage | Servo to 70° |

---

## 🛠️ Installation & Dependencies

### 1. Python (PC / Raspberry Pi)
You need ROS 2 installed (Humble/Foxy). Then install the Python libraries:

```bash
pip3 install pyserial pygame
```

### 2. Arduino
Install the following libraries via the **Arduino IDE Library Manager**:
* `AccelStepper` (by Mike McCauley)
* `Servo` (Built-in)

Upload `controller_receiver.ino` to your board. **Note:** The Baud Rate is `115200`.

### 3. Permissions
Ensure your user has permission to access the USB port:

```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```
*(Log out and back in for `usermod` to take effect).*

---

## 🚀 How to Run

### Step 1: Verify Connection (Optional but Recommended)
Before running the full ROS system, use the tester script to check if the controller inputs are reaching the Arduino correctly.

```bash
python3 controller_tester.py
```
* **Success:** You should see packet data printing in the terminal, and the Arduino LEDs/Motors should respond to button presses.
* **Failure:** Check if `/dev/ttyUSB0` is the correct port.

### Step 2: Run the ROS 2 Node
Once verified, run the actual ROS node.

```bash
# Source your ROS 2 environment
source /opt/ros/humble/setup.bash

# Run the node
python3 controller_com.py
```

---

## 📡 Serial Protocol

The system uses a custom 4-byte packet protocol for efficiency:

`[ 255, X_POS, Y_POS, BUTTON_MASK ]`

* **Header (255):** Start byte.
* **X_POS (0-254):** Right Stick X (127 is center).
* **Y_POS (0-254):** Right Stick Y (127 is center).
* **BUTTON_MASK (8-bit):**
    * Bit 0: L1
    * Bit 1: R1
    * Bit 2: L2
    * Bit 3: R2
    * Bit 4: Circle
    * Bit 5: Triangle

---

## ⚠️ Troubleshooting

1.  **"No joystick found" error:**
    * Ensure the PS4 controller is paired via Bluetooth or plugged in via USB.
    * Run `ls /dev/input/js*` to see if the device exists.

2.  **"pygame.error: Video system not initialized":**
    * The scripts utilize `os.environ["SDL_VIDEODRIVER"] = "dummy"` to run without a monitor (headless). Do not remove this line if running on a robot.

3.  **Steppers are jittery or weak:**
    * Check that the power supply provides enough current.
    * Adjust `max_speed` and acceleration in the Arduino `setup()` if the motors are stalling.
