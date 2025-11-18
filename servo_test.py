from adafruit_servokit import ServoKit
import keyboard
kit = ServoKit(channels=16)

running = True
#kit.servo[0].set_pulse_width_range(1000, 2000)
#kit.servo[1].set_pulse_width_range(1000, 2000)
#kit.servo[2].set_pulse_width_range(1000, 2000)
#kit.servo[0].angle = 90
#kit.servo[1].angle = 90
#kit.servo[2].angle = 90


def change_angle(servo, value):
    kit.continuous_servo[servo].throttle = value
    
def compactido(value):
    kit.continuous_servo[0].throttle = value
    kit.continuous_servo[1].throttle = value
    kit.continuous_servo[2].throttle = value
    kit.continuous_servo[3].throttle = value
  
    
def turn_off():
    running = False
    
keyboard.add_hotkey("a", lambda: change_angle(3,-0.1))
keyboard.add_hotkey("q", lambda: change_angle(3,0.3))
keyboard.add_hotkey("z", lambda: change_angle(3,0.1))

keyboard.add_hotkey("s", lambda: change_angle(2,-0.1))
keyboard.add_hotkey("w", lambda: change_angle(2,0.3))
keyboard.add_hotkey("x", lambda: change_angle(2,0.1))

keyboard.add_hotkey("d", lambda: change_angle(1, -0.1))
keyboard.add_hotkey("e", lambda: change_angle(1, 0.3))
keyboard.add_hotkey("c", lambda: change_angle(1,0.1))

keyboard.add_hotkey("g", lambda: compactido(-0.1))
keyboard.add_hotkey("t", lambda: compactido(0.3))
keyboard.add_hotkey("b", lambda: compactido(0.1))
keyboard.add_hotkey("s", turn_off)

while running:
    keyboard.wait()                                                                                                                                                                                            
