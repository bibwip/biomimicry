import serial
import pygame
import os
import time
import sys

# Helper function to map joystick values to 0-254
def map_joystick(val: float):
    if abs(val) < 0.2:
        val = 0
    val += 1.0
    val = val / 2.0 * 254
    return int(val + 0.5)

class ControllerTester:
    def __init__(self):
        print("Initializing Controller Tester...")

        # --- Serial Setup ---
        # NOTE: Check your port name!
        # On Linux: /dev/ttyUSB0 or /dev/ttyACM0
        # On Windows: COM3, COM4, etc.
        self.serial_port_name = "/dev/ttyUSB0"

        try:
            self.ser = serial.Serial(self.serial_port_name, 115200, timeout=1)
            self.ser.reset_input_buffer()
            print(f"Serial connected on {self.serial_port_name}")
        except Exception as e:
            print(f"Error: Could not connect to Serial: {e}")
            self.ser = None

        # --- Pygame Setup ---
        # This allows pygame to run without opening a window
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("Error: No joystick found! Connect your PS4 controller.")
            self.controller = None
            sys.exit(1) # Exit if no controller
        else:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            print(f"Controller Connected: {self.controller.get_name()}")

    def run(self):
        print("Starting main loop... Press Ctrl+C to stop.")
        try:
            while True:
                if self.controller is None:
                    break

                # Process event queue to update controller states
                pygame.event.pump()

                # Read Right Stick (mapped for Serial)
                # Note: Axis numbers might vary slightly depending on OS/Driver
                x_right = map_joystick(-self.controller.get_axis(3))
                y_right = map_joystick(self.controller.get_axis(4))

                # Read Buttons
                L1 = self.controller.get_button(4)
                R1 = self.controller.get_button(5)
                L2 = self.controller.get_button(6)
                R2 = self.controller.get_button(7)
                circle = self.controller.get_button(1)
                triangle = self.controller.get_button(2)

                # Bitwise Logic for Serial Packet
                button_data = 0b0
                if L1: button_data |= 0b0000001
                if R1: button_data |= 0b0000010
                if L2: button_data |= 0b0000100
                if R2: button_data |= 0b0001000
                if circle: button_data |= 0b0010000
                if triangle: button_data |= 0b0100000

                # Construct Packet
                # Header (255), X, Y, Buttons
                pakketje = [255, x_right, y_right, button_data]

                # Debug Prints
                print(f"Sending: {pakketje} | Bin: {bin(button_data)}")

                # Send over Serial
                if self.ser and self.ser.is_open:
                    self.ser.write(bytearray(pakketje))

                # Sleep to mimic the 0.05s timer (20Hz)
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.cleanup()

    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")
        pygame.quit()
        print("Pygame terminated.")

if __name__ == "__main__":
    tester = ControllerTester()
    tester.run()
