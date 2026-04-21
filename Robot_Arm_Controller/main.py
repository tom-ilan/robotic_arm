# USB modem ports based off location
# /dev/cu.usbmodem1201 (Top port)
# /dev/cu.usbmodem11201 (Bottom port)
import serial
import time
import math
import kinematics

i = 0

# Sets up a serial connection the ardunino via the USB modem
with serial.Serial('/dev/cu.usbmodem11201') as ser:
    while True:
        
        # Recives the inputs for the robotic arms cordinates
        x =int(100)
        y =int(0)
        z =int(100 * math.sin(i) ** 2)

        # Base servo Control
        base_input = int(kinematics.base_rotation(x, y))

        # Arm servo control
        servo_angles = kinematics.top_kinematics(x, y, z)
        top_arm_input = int(servo_angles[0]) 
        bottom_arm_input = int(servo_angles[1])

        # Sends the inputs to the arm
        ser.write(bytes([top_arm_input, base_input, bottom_arm_input]))
        i += 0.02
        time.sleep(0.05)

