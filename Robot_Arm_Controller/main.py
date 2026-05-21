# USB modem ports based off location
# /dev/cu.usbmodem1201 (Top port)
# /dev/cu.usbmodem11201 (Bottom port)
import serial
from num_packer import num_packer
import go_to
import math
import time
i = 0

# Sets up a serial connection the ardunino via the USB modem
with serial.Serial('/dev/cu.usbmodem1101') as ser:
    while True:
        go_to.go_to_precise(ser, x_mm = int(input('x: ')), y_mm = int(input('y: ')), z_mm = int(input('z: ')))


