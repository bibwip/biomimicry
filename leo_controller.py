import serial
import pygame
import os
import time

# --- SETUP: Headless Mode ---
# This line allows pygame to run without a monitor attached (common for Pi projects)
os.environ["SDL_VIDEODRIVER"] = "dummy"

# --- SETUP: Arduino Serial ---
# Ensure this matches your port (check 'ls /dev/tty*' in terminal)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.reset_input_buffer()

# --- SETUP: PS4 Controller ---
pygame.init()
pygame.joystick.init()

# Check if controller is connected
if pygame.joystick.get_count() == 0:
    print("No controller found! Connect via USB or Bluetooth.")
    exit()

# Initialize the first controller found
controller = pygame.joystick.Joystick(0)
controller.init()

print(f"Connected to: {controller.get_name()}")
print("Press 'X' (Button 0) to light the LED.")
print("Press CTRL+C to quit.")

try:
    while True:
        # Check for controller events
        for event in pygame.event.get():

            # Button Pressed Down
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0: # 0 is usually the 'X' button on PS4
                    print("X Pressed -> LED ON")
                    ser.write(b'1')

            # Button Released
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == 0:
                    print("X Released -> LED OFF")
                    ser.write(b'0')

        time.sleep(0.01) # Small delay to prevent high CPU usage

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
    pygame.quit()
