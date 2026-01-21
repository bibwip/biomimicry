# ----------------------------------------------------------------------------
# File: test_controller.py
#
# Description:
#   A standalone test script to test the PS4 Controller -> Serial connection
#   WITHOUT needing ROS or the Leo Rover software stack.
#
#   Use this to:
#   1. Verify your PS4 controller is paired and detected by Linux/Pygame.
#   2. Verify the Serial connection to the Arduino/Microcontroller works.
#   3. View the raw data packet (list of 4 bytes) being sent in real-time.
#
#   Protocol: [Header (255), RightStick_X, RightStick_Y, Button_Bitmask]
# ----------------------------------------------------------------------------

import serial
import pygame
import os
import time
import sys


def map_joystick(val: float):
    """
    Maps joystick values (-1.0 to 1.0) to a byte-safe integer range (0 to 254).
    255 is excluded because it is used as the Serial Start Byte.
    """
    if abs(val) < 0.2:
        val = 0

    val += 1.0
    val = val / 2.0 * 254
    return int(val + 0.5)


class ControllerTester:
    def __init__(self):
        print("Initializing Controller Tester...")

        # --- Serial Setup ---
        # NOTE: Update this port if you are on Windows (e.g., "COM3")
        # or if your USB device mounts differently (e.g., "/dev/ttyACM0")
        self.serial_port_name = "/dev/ttyUSB0"

        try:
            self.ser = serial.Serial(self.serial_port_name, 115200, timeout=1)
            self.ser.reset_input_buffer()
            print(f"SUCCESS: Serial connected on {self.serial_port_name}")
        except Exception as e:
            print(f"ERROR: Could not connect to Serial: {e}")
            print("Check cables and permissions (sudo chmod 666 /dev/ttyUSB0)")
            self.ser = None

        # --- Pygame Setup ---
        # "dummy" driver allows Pygame to run without a monitor/GUI window.
        # Essential for running via SSH.
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.joystick.init()

        # Check for controller
        if pygame.joystick.get_count() == 0:
            print("ERROR: No joystick found! Connect your PS4 controller via Bluetooth or USB.")
            self.controller = None
            sys.exit(1)  # Force exit if hardware is missing
        else:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            print(f"SUCCESS: Controller Connected: {self.controller.get_name()}")

    def run(self):
        print("\nStarting main loop... Press Ctrl+C to stop.")
        try:
            while True:
                if self.controller is None:
                    break

                pygame.event.pump()

                # 2. Read Right Stick (Mapped for Manipulator Control)
                # Note: Axis 3/4 are usually Right Stick X/Y on standard mappings
                x_right = map_joystick(-self.controller.get_axis(3))
                y_right = map_joystick(self.controller.get_axis(4))

                # 3. Read Digital Buttons
                # Note: Button IDs can vary by OS. These are standard PS4 mappings.
                L1 = self.controller.get_button(4)
                R1 = self.controller.get_button(5)
                L2 = self.controller.get_button(6)
                R2 = self.controller.get_button(7)
                circle = self.controller.get_button(1)
                triangle = self.controller.get_button(2)

                # 4. Pack Buttons into Bitmask
                # This compresses 6 booleans into 1 byte to save bandwidth
                button_data = 0b0
                if L1:       button_data |= 0b0000001
                if R1:       button_data |= 0b0000010
                if L2:       button_data |= 0b0000100
                if R2:       button_data |= 0b0001000
                if circle:   button_data |= 0b0010000
                if triangle: button_data |= 0b0100000

                # 5. Construct Packet: [Header, X, Y, Bits]
                pakketje = [255, x_right, y_right, button_data]

                # 6. Debug Output
                # Show exactly what is being sent. 'bin()' helps visualize the bitmask.
                print(f"Sending: {pakketje} | Bits: {bin(button_data)}")

                # 7. Send over Serial
                if self.ser and self.ser.is_open:
                    self.ser.write(bytearray(pakketje))

                # 8. Loop Timing
                # 0.05s = 20Hz (Updates 20 times per second)
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nStopping by user request...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Safe shutdown of resources."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")
        pygame.quit()
        print("Pygame terminated.")


if __name__ == "__main__":
    tester = ControllerTester()
    tester.run()
