import serial
import go_to


# Sets up a serial connection the ardunino via the USB modem
with serial.Serial(input('Serial Port:')) as ser:
    while True:
        go_to.go_to_angle(ser, base = int(input('base: ')), bottom = int(input('bottom: ')), top = int(input('top: ')), gripper_degrees = int(input('gripper: ')))


