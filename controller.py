from pyPS4Controller.controller import Controller
from adafruit_servokit import ServoKit
from threading import Thread
import numpy as np
from time import sleep
kit = ServoKit(channels=16)


# Controller class that listens and handles the presses of the playstation
# controller.
class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.min_value = 3276
        self.max_value = 32766

        self.X = 0
        self.Y = 0
        self.Z = 0

        self.gripper_angle = 0
        self.gripper_speed = 0
        self.gripper_max_speed = 5

        self.scoop_speed = 0

    def on_R3_left(self, value):
        if abs(value) > self.min_value:
            self.X = value/self.max_value
        else:
            self.X = 0

    def on_R3_right(self, value):
        if abs(value) > self.min_value:
            self.X = value/self.max_value
        else:
            self.X = 0

    def on_R3_up(self, value):
        if abs(value) > self.min_value:
            self.Y = value/self.max_value
        else:
            self.Y = 0

    def on_R3_down(self, value):
        if abs(value) > self.min_value:
            self.Y = value/self.max_value
        else:
            self.Y = 0

    def on_R2_press(self, value):
        if abs(value) > self.min_value:
            self.Z = abs(value)/self.max_value

    def on_R2_release(self):
        self.Z = 0

    def on_L2_press(self, value):
        if abs(value) > self.min_value:
            self.Z = -abs(value)/self.max_value

    def on_L2_release(self):
        self.Z = 0

    def on_L3_down(self, value):
        factor = abs(value)/self.max_value
        self.gripper_speed = -(factor * self.gripper_max_speed)


    def on_L3_up(self, value):
        factor = abs(value)/self.max_value
        self.gripper_speed = factor * self.gripper_max_speed

    def on_L3_y_at_rest(self):
        self.gripper_speed = 0

    def on_up_arrow_press(self):
        self.scoop_speed = 0.3

    def on_down_arrow_press(self):
        self.scoop_speed = -0.3

    def on_up_down_arrow_release(self):
        self.scoop_speed = 0

    # Calculates the throttle speed of each servo to move the arm a certain
    # direction.
    def calculate_servo(self):
        while True:

            if self.gripper_speed != 0:
                self.gripper_angle += self.gripper_speed
                if self.gripper_angle > 180: self.gripper_angle = 180
                if self.gripper_angle < 0: self.gripper_angle = 0

                kit.servo[3].angle = self.gripper_angle

            serv1 = 0
            serv2 = 0
            serv3 = 0

            power = np.sqrt(self.Y**2 + self.X**2)
            angle = (np.pi + np.atan2(-self.X, self.Y)) * 180 / np.pi;

            if angle < 120:
                ratio = angle/120
                serv2 = ratio*power
                serv1 = (1-ratio)*power
                serv3 = 0
            elif angle < 240:
                angle -= 120
                ratio = angle/120
                serv3 = ratio*power
                serv2 = (1-ratio)*power
                serv1 = 0
            else:
                angle -= 240
                ratio = angle/120
                serv1 = ratio*power
                serv3 = (1-ratio)*power
                serv2 = 0

            sleep(0.05)
            serv1 += self.Z/2 + 0.1
            serv2 += self.Z/2 + 0.1
            serv3 += self.Z/2 + 0.1
            scoop_serv = self.scoop_speed + 0.1
            print(serv1,serv2,serv3)
            kit.continuous_servo[0].throttle = serv1
            kit.continuous_servo[1].throttle = serv2
            kit.continuous_servo[2].throttle = serv3
            kit.continuous_servo[4].throttle = scoop_serv


controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
servoding = Thread(target= controller.calculate_servo)

servoding.start()
controller.listen()
