import serial
import pygame
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


def map_joystick(value: float):
    val += 1
    val = val / 2.0 * 254
    return int(val + 0.5)


class ControllerCom(Node):
    def __init__(self):
        super().__init__("ControllerCom")
        # Create ROS 2 publisher
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f"Could not connect to Serial: {e}")
            self.ser = None

        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        self.deadzone = 0.1

        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.joystick.init()

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

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.controller is None:
            return

        pygame.event.pump()

        x_left = -self.controller.get_axis(0)
        y_left = -self.controller.get_axis(1)

        x_right = map_joystick(-self.controller.get_axis(2))
        y_right = map_joystick(-self.controller.get_axis(3))

        L1 = self.controller.get_button(4)
        R1 = self.controller.get_button(5)
        L2 = self.controller.get_button(6)
        R2 = self.controller.get_button(7)
        circle = self.controller.get_button(2)
        triangle = self.controller.get_button(3)

        button_data = 0
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

        pakketje = [255, x_right, y_right, button_data]
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


# --- SETUP: Arduino Serial ---
# Ensure this matches your port (check 'ls /dev/tty*' in terminal)


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
