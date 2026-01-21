'''
Controller Communication Node

This program reads the PS4 controller inputs via Pygame and sends:
1. Velocity commands to ROS2 for robot movement.
2. Serial packets to an arduino for control of the arm.

'''
import serial
import pygame
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Maps joystick value (-1.0 to 1.0) to integer (0 to 254)
def map_joystick(val: float):
    if abs(val) < 0.2:
        val = 0
    val += 1.0
    val = val / 2.0 * 254
    return int(val + 0.5)


class ControllerCom(Node):
    def __init__(self):
        super().__init__("ControllerCom")
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Setup Serial connection to Arduino
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f"Could not connect to Serial: {e}")
            self.ser = None

        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        self.deadzone = 0.1

        # Set SDL to dummy to allow Pygame to run without a monitor/window
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.joystick.init()

        # Connect to the first available joystick
        if pygame.joystick.get_count() == 0:
            self.get_logger().error(
                "No joystick found! Connect your PS4 controller."
            )
            self.controller = None
        else:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            self.get_logger().info(
                f"Controller Connected: {self.controller.get_name()}"
            )

        # Run the loop every 0.05 seconds (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.controller is None:
            return

        pygame.event.pump()

        # Left stick for Robot Movement
        x_left = -self.controller.get_axis(0)
        y_left = -self.controller.get_axis(1)

        # Right stick for Serial Device (mapped to 0-254)
        x_right = map_joystick(-self.controller.get_axis(3))
        y_right = map_joystick(self.controller.get_axis(4))

        L1 = self.controller.get_button(4)
        R1 = self.controller.get_button(5)
        L2 = self.controller.get_button(6)
        R2 = self.controller.get_button(7)
        circle = self.controller.get_button(1)
        triangle = self.controller.get_button(2)

        # Compress buttons into a single byte using bitwise OR
        button_data = 0b0
        if L1:
            button_data |= 0b0000001
        if R1:
            button_data |= 0b0000010
        if L2:
            button_data |= 0b0000100
        if R2:
            button_data |= 0b0001000
        if circle:
            button_data |= 0b0010000
        if triangle:
            button_data |= 0b0100000

        # Create packet: [StartByte (255), X, Y, Buttons]
        pakketje = [255, x_right, y_right, button_data]
        if self.ser:
            self.ser.write(pakketje)

        twist = Twist()

        
        if abs(y_left) > self.deadzone:
            twist.linear.x = y_left * self.max_linear_speed
        else:
            twist.linear.x = 0.0

        if abs(x_left) > self.deadzone:
            twist.angular.z = x_left * self.max_angular_speed
        else:
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

        if twist.linear.x != 0 or twist.angular.z != 0:
            self.get_logger().info(
                f"Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}"
            )


def main():
    rclpy.init()
    node = ControllerCom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == "__main__":
    main()
