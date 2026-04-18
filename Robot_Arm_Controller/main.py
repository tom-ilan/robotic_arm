# USB modem ports based off location
# /dev/cu.usbmodem1201 (Top port)
# /dev/cu.usbmodem11201 (Bottom port)

import serial
import time

# Sets up a serial connection the ardunino via the USB modem
with serial.Serial('/dev/cu.usbmodem11201') as ser:
    while True:
        # Recives the inputs for the robotic arm angles
        servoOneInput = int(input("Angle for servo one: "))
        servoTwoInput = int(input("Angle for servo two: "))
        servoThreeInput = int(input("Angle for servo three: "))

        # Sends the inputs to the arm
        ser.write(bytes([servoOneInput, servoTwoInput, servoThreeInput]))

