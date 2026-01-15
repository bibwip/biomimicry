#include <Servo.h>
#include <math.h>
#include <AccelStepper.h>


#define X_STEP 2
#define X_DIR 5

#define Y_STEP 3
#define Y_DIR 6

#define Z_STEP 4
#define Z_DIR 7

#define SERVO_CLAW A0
#define SERVO_STOR A1

AccelStepper stepper1(1, Y_STEP, Y_DIR);
AccelStepper stepper2(1, X_STEP, X_DIR);
AccelStepper stepper3(1, Z_STEP, Z_DIR);

Servo serv_claw;
Servo serv_stor;

int joyX = 127;
int joyY = 127;
byte buttonState = 0;

int claw_pos = 0;
int serv_speed = 5;

float max_speed = 25.0;

void setup() {
  Serial.begin(115200);
  serv_claw.attach(SERVO_CLAW);
  serv_stor.attach(SERVO_STOR);
  pinMode(LED_BUILTIN, OUTPUT);

  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(1000);
  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(1000);
  stepper3.setMaxSpeed(2000);
  stepper3.setAcceleration(1000);
}

void calculate_motor_speeds(double X, double Y, double z_factor) {
  float motor1 = 0;
  float motor2 = 0;
  float motor3 = 0;

  double power = sqrt(X * X + Y * Y);
  double angle = (PI + atan2(-X, Y)) * 180 / PI;
  double ratio;

  if (angle < 120) {
    ratio = angle / 120;
    motor1 = (1 - ratio) * power;
    motor2 = ratio * power;
    motor3 = 0;
  } else if (angle < 240) {
    angle -= 120;
    ratio = angle / 120;
    motor1 = 0;
    motor2 = (1 - ratio) * power;
    motor3 = ratio * power;
  } else {
    angle -= 240;
    ratio = angle / 120;
    motor1 = ratio * power;
    motor2 = 0;
    motor3 = (1 - ratio) * power;
  }

  motor1 += z_factor;
  motor2 += z_factor;
  motor3 += z_factor;


  if (abs(motor1) > 0.15) stepper1.move(motor1 * max_speed);
  if (abs(motor2) > 0.15) stepper2.move(motor2 * max_speed);
  if (abs(motor3) > 0.15) stepper3.move(motor3 * max_speed);
}


void loop() {
  if (Serial.available() > 10) {
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  if (Serial.available() > 0) {
    if (Serial.read() == 255) {
      while (Serial.available() < 3) {}

      joyX = Serial.read();
      joyY = Serial.read();
      buttonState = Serial.read();

      double finalX = (joyX / 127.0) - 1.0;
      double finalY = (joyY / 127.0) - 1.0;

      if (abs(finalX) < 0.2) { finalX = 0; }
      if (abs(finalY) < 0.2) { finalY = 0; }

      int finalZ = 0;

      if ((buttonState & 0b000001) != (buttonState & 0b000010)) {
        // L1 - claw close
        if (buttonState & 0b000001) {
          claw_pos += serv_speed;
          if (claw_pos > 180) { claw_pos = 180; }
          serv_claw.write(claw_pos);
        }

        // R1 - claw open
        if (buttonState & 0b000010) {
          claw_pos -= serv_speed;
          if (claw_pos < 0) { claw_pos = 0; }
          serv_claw.write(claw_pos);
        }
      }

      if ((buttonState & 0b000100) != (buttonState & 0b001000)) {
        // L2 - Arm retract
        if (buttonState & 0b000100) {
          finalZ = -1;
          digitalWrite(LED_BUILTIN, HIGH);
        }

        // R2 - Arm extend
        if (buttonState & 0b001000) {
          finalZ = 1;
          digitalWrite(LED_BUILTIN, LOW);
        }
      }

      if ((buttonState & 0b010000) != (buttonState & 0b100000)) {
        // Circle - Storage open
        if (buttonState & 0b010000) {
          serv_stor.write(110);
        }

        // Triangle - Storage close
        if (buttonState & 0b100000) {
          serv_stor.write(70);
        }
      } else {
        serv_stor.write(90);
      }

      calculate_motor_speeds(finalX, finalY, finalZ);
    }
  }
  stepper1.run();
  stepper2.run();
  stepper3.run();
}
