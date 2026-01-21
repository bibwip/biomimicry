# ROS 2 & Arduino arm Control System

This project implements a teleoperation system for the leo rover equipped with a custom arm that is controlled with 3 steppemotors. It uses a PS4 Controller to simultaneously drive the robot via ROS 2 and control the manipulator via a Serial connection to an Arduino.

## File Overview

| File | Description |
| :--- | :--- |
| `leo_controller.py` | **ROS 2 Node.** Reads PS4 inputs, publishes `cmd_vel` to the robot, and sends Serial packets to the Arduino. |
| `controller_receiver.ino` | **Arduino Firmware.** Receives Serial packets, calculates inverse kinematics, and drives Steppers/Servos. |
| `test_controller.py` | **Diagnostic Script.** Tests the Controller-to-Arduino connection without needing ROS. |

---

## Hardware Setup

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

## Controls Mapping

The system uses a **"Split Control"** scheme:
* **Left Stick:** Drives the Robot base (ROS 2).
* **Right Stick & Buttons:** Controls the Manipulator (Arduino).

| Input | Action | Logic |
| :--- | :--- | :--- |
| **Left Stick Y** | Robot Forward/Back | ROS `cmd_vel.linear.x` |
| **Left Stick X** | Robot Rotate Left/Right | ROS `cmd_vel.angular.z` |
| **Right Stick** | Move Arm | Mapped to Stepper kinematic X/Y |
| **L1** | Close Claw | Servo moves + |
| **R1** | Open Claw | Servo moves - |
| **L2** | Arm Retract (Down) | Z-axis -1 |
| **R2** | Arm Extend (Up) | Z-axis +1 |
| **Circle** | Open Storage | Servo moves + |
| **Triangle** | Close Storage | Servo moves - |

---

## Installation & Dependencies

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

## How to Run

### Step 1: Load Scripts onto the Rover
If you are connected to the rover via SSH and need to create the file directly, use `cat`:

1.  **Create the file:**
    ```bash
    cat > controller_com.py
    ```
2.  **Paste** the contents of your Python script into the terminal.
3.  Press **Ctrl+D** to save and exit.

### Step 2: Make Executable
Change the file permission to allow it to run as a program:

```bash
chmod +x controller_com.py
```

### Step 3: Execute
Now you can run the script directly:

```bash
python3 controller_com.py
```

## Serial Protocol

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

3.  **Steppers are jittery or weak:**
    * Check that the power supply provides enough current.
    * Adjust `max_speed` and acceleration in the Arduino `setup()` if the motors are stalling.
