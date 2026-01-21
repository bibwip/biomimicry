/*
 * ----------------------------------------------------------------------------
 * File: controller_reciever.ino
 * Description:
 * Firmware for the movement of the arm, claw and storage.
 * controlled via Serial commands from a host computer (ROS 2 Python node).
 * * Hardware:
 * - 3x Stepper Motors (X, Y, Z axes) using AccelStepper.
 * - 2x Servo Motors (Claw, Storage).
 * * Serial Protocol (4 Bytes):
 * [Header (255), JoyX (0-254), JoyY (0-254), Buttons (Bitmask)]
 * ----------------------------------------------------------------------------
 */

#include <Servo.h>
#include <math.h>
#include <AccelStepper.h>

// --- Pin Definitions ---
// Driver Pins for Stepper Motors (likely A4988 or DRV8825 drivers)
#define X_STEP 2
#define X_DIR 5

#define Y_STEP 3
#define Y_DIR 6

#define Z_STEP 4
#define Z_DIR 7

// Servo Pins
#define SERVO_CLAW A0
#define SERVO_STOR A1

// --- Object Initialization ---
// Interface type 1 = Stepper Driver (Step + Direction pins)
AccelStepper stepper1(1, Y_STEP, Y_DIR);
AccelStepper stepper2(1, X_STEP, X_DIR);
AccelStepper stepper3(1, Z_STEP, Z_DIR);

Servo serv_claw;
Servo serv_stor;

// --- State Variables ---
int joyX = 127;          // Center point for 0-254 range
int joyY = 127;
byte buttonState = 0;    // Bitmask storage for buttons

int claw_pos = 0;        // Current servo position (0-180)
int serv_speed = 5;      // How many degrees to move per loop cycle

float max_speed = 25.0;  // Multiplier for stepper movement distance

void setup() {
  Serial.begin(115200); // Must match the baud rate in the Python script

  // Attach Servos
  serv_claw.attach(SERVO_CLAW);
  serv_stor.attach(SERVO_STOR);

  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(1000);

  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(1000);

  stepper3.setMaxSpeed(2000);
  stepper3.setAcceleration(1000);
}

/**
 * Calculates motor movements based on joystick input.
 * * This algorithm splits the 360-degree joystick input into three 120-degree sectors.
 * It distributes power to the 3 motors based on the angle.
 * * Args:
 * X, Y: Normalized joystick inputs (-1.0 to 1.0)
 * z_factor: Up/Down movement (-1, 0, or 1)
 */
void calculate_motor_speeds(double X, double Y, double z_factor) {
  float motor1 = 0;
  float motor2 = 0;
  float motor3 = 0;

  // Calculate Joystick Magnitude (Power) and Direction (Angle)
  double power = sqrt(X * X + Y * Y);
  // atan2 returns radians, convert to degrees. Offset by PI to handle -180 to 180 range.
  double angle = (PI + atan2(-X, Y)) * 180 / PI;
  double ratio;

  // --- Sector 1: 0 to 120 degrees ---
  if (angle < 120) {
    ratio = angle / 120;
    motor1 = (1 - ratio) * power;
    motor2 = ratio * power;
    motor3 = 0;
  }
  // --- Sector 2: 120 to 240 degrees ---
  else if (angle < 240) {
    angle -= 120;
    ratio = angle / 120;
    motor1 = 0;
    motor2 = (1 - ratio) * power;
    motor3 = ratio * power;
  }
  // --- Sector 3: 240 to 360 degrees ---
  else {
    angle -= 240;
    ratio = angle / 120;
    motor1 = ratio * power;
    motor2 = 0;
    motor3 = (1 - ratio) * power;
  }

  // Add the Z-axis component (Up/Down)
  motor1 += z_factor;
  motor2 += z_factor;
  motor3 += z_factor;

  // Command the steppers to move relative to current position
  // We use a threshold (0.15) to prevent jitter from noise
  if (abs(motor1) > 0.15) stepper1.move(motor1 * max_speed);
  if (abs(motor2) > 0.15) stepper2.move(motor2 * max_speed);
  if (abs(motor3) > 0.15) stepper3.move(motor3 * max_speed);
}


void loop() {
  // --- 1. Buffer Cleanup ---
  // If the buffer is too full (lagging), clear it to get the latest packet.
  if (Serial.available() > 10) {
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  // --- 2. Serial Reading ---
  if (Serial.available() > 0) {
    // Check for Start Byte (Header) = 255
    if (Serial.read() == 255) {

      // Wait for the remaining 3 bytes (X, Y, Buttons).
      while (Serial.available() < 3) {}

      // Read Payload
      joyX = Serial.read();
      joyY = Serial.read();
      buttonState = Serial.read();

      // Normalize Joystick inputs (0-254) -> (-1.0 to 1.0)
      double finalX = (joyX / 127.0) - 1.0;
      double finalY = (joyY / 127.0) - 1.0;

      // Apply Deadzone
      if (abs(finalX) < 0.2) { finalX = 0; }
      if (abs(finalY) < 0.2) { finalY = 0; }

      // --- 3. Button Logic (Bitmask Decoding) ---
      int finalZ = 0;

      // Check L1 (Bit 0) and R1 (Bit 1) for Claw
      if ((buttonState & 0b000001) != (buttonState & 0b000010)) {
        // L1 - Claw Close
        if (buttonState & 0b000001) {
          claw_pos += serv_speed;
          if (claw_pos > 180) { claw_pos = 180; }
          serv_claw.write(claw_pos);
        }

        // R1 - Claw Open
        if (buttonState & 0b000010) {
          claw_pos -= serv_speed;
          if (claw_pos < 0) { claw_pos = 0; }
          serv_claw.write(claw_pos);
        }
      }

      // Check L2 (Bit 2) and R2 (Bit 3) for Z-Axis (Arm height)
      if ((buttonState & 0b000100) != (buttonState & 0b001000)) {
        // L2 - Retract Arm (Z down)
        if (buttonState & 0b000100) {
          finalZ = -1;
        }

        // R2 - Extend Arm (Z up)
        if (buttonState & 0b001000) {
          finalZ = 1;
        }
      }

      // Check Circle (Bit 4) and Triangle (Bit 5) for Storage Bin
      if ((buttonState & 0b010000) != (buttonState & 0b100000)) {
        // Circle - Open Storage
        if (buttonState & 0b010000) {
          serv_stor.write(110);
        }

        // Triangle - Close Storage
        if (buttonState & 0b100000) {
          serv_stor.write(70);
        }
      } else {
        // Default / Idle position for storage
        serv_stor.write(90);
      }

      // Send calculated targets to Steppers
      calculate_motor_speeds(finalX, finalY, finalZ);
    }
  }

  // --- 4. Motor Execution ---
  // Must be called as frequently as possible to generate step pulses.
  stepper1.run();
  stepper2.run();
  stepper3.run();
}
