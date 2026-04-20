# USB modem ports based off location
# /dev/cu.usbmodem1201 (Top port)
# /dev/cu.usbmodem11201 (Bottom port)
import serial
import time
import math
import kinematics

# Sets up a serial connection the ardunino via the USB modem
with serial.Serial('/dev/cu.usbmodem11201') as ser:
    while True:
        # Recives the inputs for the robotic arms cordinates
        x =int(input("x :"))
        y =int(input("y :"))
        z =int(input("z :"))

        servoTwoInput = int(math.degrees(kinematics.base_rotation(x, y)))

        servo_angles = kinematics.top_kinematics(x, y, z)

        servoOneInput = int(servo_angles[0]) 
        servoThreeInput = int(servo_angles[1])

        # Sends the inputs to the arm
        ser.write(bytes([servoOneInput, servoTwoInput, servoThreeInput]))
        time.sleep(0.05)

