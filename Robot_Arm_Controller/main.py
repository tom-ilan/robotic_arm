# USB modem ports based off location
# /dev/cu.usbmodem1201 (Top port)
# /dev/cu.usbmodem11201 (Bottom port)
import serial
import time
import math
import kinematics

i = 0
def go_to(ser, x_mm: float, y_mm: float, z_mm: float):
     # Base servo Control
        base_degrees, bottom_degrees, top_degrees = [
            int(math.degrees(radians)) for radians in kinematics.get_robot_angles_degrees(x_mm, y_mm, z_mm)
        ]

        ser.write(bytes([top_degrees, base_degrees, bottom_degrees]))

# Sets up a serial connection the ardunino via the USB modem
with serial.Serial('/dev/cu.usbmodem11201') as ser:
    while True:
        go_to(ser, x_mm=0, y_mm=0, z_mm=0)

        for()

