import serial
import go_to

#Sets up a serial connection to the ardunino
with serial.Serial(input('Serial Port:')) as ser:
    while True:
        go_to.go_to_mm(ser, x_mm = int(input('x: ')), y_mm = int(input('y: ')), z_mm = int(input('z: ')), gripper_degrees = int(input('gripper: ')))


