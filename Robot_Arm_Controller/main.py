# USB modem ports based off location
# /dev/cu.usbmodem1201 (Top port)
# /dev/cu.usbmodem11201 (Bottom port)
import serial
import time
import math
import kinematics
from num_packer import num_packer

i = 0
def go_to(ser, x_mm: float, y_mm: float, z_mm: float):

     # Base servo Control
        # get_robot_angles_degrees returns radians — convert to degrees before packing
        base_rad, bottom_rad, top_rad = kinematics.get_robot_angles_degrees(x_mm, y_mm, z_mm)
        base_degrees = math.degrees(base_rad)
        bottom_degrees = math.degrees(bottom_rad)
        top_degrees = math.degrees(top_rad)
 
        base_degrees_packed_big, base_degrees_packed_little = num_packer(base_degrees)
        bottom_degrees_packed_big, bottom_degrees_packed_little = num_packer(bottom_degrees)
        top_degrees_packed_big, top_degrees_packed_little = num_packer(top_degrees)

        # Debug: show centi-degrees and bytes sent
        base_centi = int(round(base_degrees * 100))
        bottom_centi = int(round(bottom_degrees * 100))
        top_centi = int(round(top_degrees * 100))
        print("Python -> base_centi:", base_centi, "bytes:", base_degrees_packed_big, base_degrees_packed_little)
        print("Python -> bottom_centi:", bottom_centi, "bytes:", bottom_degrees_packed_big, bottom_degrees_packed_little)
        print("Python -> top_centi:", top_centi, "bytes:", top_degrees_packed_big, top_degrees_packed_little)
 
        ser.write(bytes([base_degrees_packed_big, base_degrees_packed_little, bottom_degrees_packed_big, bottom_degrees_packed_little, top_degrees_packed_big, top_degrees_packed_little]))
        ser.flush()
        time.sleep(0.01)

# Sets up a serial connection the ardunino via the USB modem
with serial.Serial('/dev/cu.usbmodem1101') as ser:
    while True:
        go_to(ser, x_mm = int(input('x: ')), y_mm = int(input('y: ')), z_mm = int(input('z: ')))



