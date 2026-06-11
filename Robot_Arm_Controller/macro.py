import time
import go_to
import serial
import math
import pandas as pd

macro = pd.read_csv('Robot_Arm_Controller/__test_locations.csv')

print(macro.to_numpy().tolist())