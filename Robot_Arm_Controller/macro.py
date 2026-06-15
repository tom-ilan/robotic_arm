import time
import go_to
import serial
import math
import pandas as pd

# Opens the macro csv and converts each row to a list
macro_csv = pd.read_csv('Robot_Arm_Controller/__test_locations.csv')
macro = macro_csv.to_numpy().tolist()
print(macro)

def run_macro(location_list: list):
    with serial.Serial('/dev/cu.usbmodem1101') as ser:
        for location in location_list:
            x_mm, y_mm, z_mm, gripper_degrees, seconds = location
            go_to.go_to_mm(ser, x_mm, y_mm, z_mm, gripper_degrees)
            time.sleep(seconds)
time.sleep(2) # Sleep for 2 seconds before starting the macro
run_macro(macro)