from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

kit.servo[3].set_pulse_width_range(1000, 2000)
kit.servo[3].angle = 90


while True:
    print(kit.servo[3].angle)
    var = int(input("angle:"))
    kit.servo[3].angle = var
