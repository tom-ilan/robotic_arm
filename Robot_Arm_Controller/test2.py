# USB modem ports based off location
# /dev/cu.usbmodem1201 (Top port)
# /dev/cu.usbmodem11201 (Bottom port)
import serial
import time
import math
import kinematics
from num_packer import num_packer

i = 0
def go_to(ser, top: float, bottom: float, base: float):
 
        base_degrees_packed_big, base_degrees_packed_little = num_packer(base)
        bottom_degrees_packed_big, bottom_degrees_packed_little = num_packer(bottom)
        top_degrees_packed_big, top_degrees_packed_little = num_packer(top)
 
        ser.write(bytes([base_degrees_packed_big, base_degrees_packed_little, bottom_degrees_packed_big, bottom_degrees_packed_little, top_degrees_packed_big, top_degrees_packed_little]))
# Sets up a serial connection the ardunino via the USB modem
with serial.Serial('/dev/cu.usbmodem1101') as ser:
    while True:
        go_to(ser, top = float(input('top: ')), bottom = float(input('bottom: ')), base = float(input('base: ')))



