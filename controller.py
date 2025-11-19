from pyPS4Controller.controller import Controller
# from adafruit_servokit import ServoKit
from threading import Thread
import numpy as np
from time import sleep
# kit = ServoKit(channels=16)


X = 0
Y = 0
Z = 0
MAX = 32766


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.min_value = 3276

    def on_R3_left(self, value):
        global X
        if abs(value) > self.min_value:
            X = value/MAX
        else:
            X = 0

    def on_R3_right(self, value):
        global X
        if abs(value) > self.min_value:
            X = value/MAX
        else:
            X = 0

    def on_R3_up(self, value):
        global Y
        if abs(value) > self.min_value:
            Y = value/MAX
        else:
            Y = 0

    def on_R3_down(self, value):
        global Y
        if abs(value) > self.min_value:
            Y = value/MAX
        else:
            Y = 0


def calculate_servo():
    while True:
        serv1 = 0
        serv2 = 0
        serv3 = 0

        power = np.sqrt(Y**2 + X**2)
        angle = (np.pi + np.atan2(-X, Y)) * 180 / np.pi;

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
        sleep(0.3)
        print(serv1,serv2,serv3)
        # kit.continuous_servo[servo].throttle = value

servoding = Thread(target= calculate_servo)
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)

servoding.start()
controller.listen()
