#include <Servo.h>
#include <math.h>

Servo serv_claw;
Servo serv_stor;

int joyX = 127;
int joyY = 127;
byte buttonState = 0;

int claw_pos = 0;
int stor_pos = 0;
int serv_speed = 5;

double arm1 = 0;
double arm2 = 0;
double arm3 = 0;
double armZ = 0;

void calculate_servo(int X, int Y) {
  double power = sqrt(X ^ 2 + Y ^ 2);
  double angle = (M_PI + atan2(-X, Y)) * 180 / M_PI;
  double ratio;
  if (angle < 120) {
    ratio = angle / 120;
    arm1 = (1 - ratio) * power;
    arm2 = ratio * power;
    arm3 = 0;
  } else if (angle < 240) {
    angle -= 120;
    ratio = angle / 120;
    arm1 = 0;
    arm2 = (1 - ratio) * power;
    arm3 = ratio * power;
  } else {
    angle -= 240;
    ratio = angle / 120;
    arm1 = ratio * power;
    arm2 = 0;
    arm3 = (1 - ratio) * power;
  }
  arm1 += armZ;
    arm2 += armZ;
      arm3 += armZ;
}

void setup() {
  Serial.begin(9600);
  serv_claw.attach(9);
  serv_stor.attach(8);
}

void loop() {
  if (Serial.available()) {
    if (Serial.read() == 255) {
      while (Serial.available() < 3) {
        joyX = Serial.read();
        joyY = Serial.read();
        buttonState = Serial.read();

        if (buttonState & 0b000001 != buttonState & 0b000010) {
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

        if (buttonState & 0b000100 != buttonState & 0b001000) {
          // L2 - Arm retract
          if (buttonState & 0b000100) {
            armZ = 1;
          }

          // R2 - Arm extend
          if (buttonState & 0b001000) {
            armZ = -1;
          }
        }

        if (buttonState & 0b010000 != buttonState & 0b100000) {
          // Circle - Storage open
          if (buttonState & 0b010000) {
            stor_pos += serv_speed;
            serv_stor.write(stor_pos);
          }

          // Triangle - Storage close
          if (buttonState & 0b100000) {
            stor_pos -= serv_speed;
            serv_stor.write(stor_pos);
          }
        }

        calculate_servo(joyX, joyY);
      }
    }
  }
}
