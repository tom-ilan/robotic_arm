import os
import time
import math
import pandas as pd
import serial
import glide_to
import kinematics

# Resolve the CSV file path relative to this script's directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(SCRIPT_DIR, input('CSV Path: '))

# Opens the macro csv and converts each row to a list
macro_csv = pd.read_csv(CSV_PATH)
macro = macro_csv.to_numpy().tolist()

# Open serial port once outside the loop to prevent resetting the Arduino on every iteration
with serial.Serial('/dev/cu.usbmodem1101', timeout=1.0) as ser:
    # Allow 2 seconds for Arduino bootloader to initialize after DTR line reset
    time.sleep(2)
    
    # Read the current servo angles printed by the Arduino in setup()
    current_angles = None
    print("Reading startup angles from Arduino...")
    
    # Read up to 20 lines to find the INIT: prefix
    for _ in range(20):
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith("INIT:"):
            try:
                parts = line.split("INIT:")[1].split(",")
                current_angles = [float(p) for p in parts]
                if len(current_angles) == 4:
                    break
            except Exception:
                pass

    if not current_angles or len(current_angles) != 4:
        current_angles = [90.0, 90.0, 90.0, 90.0]
        print(f"Fallback to default initial angles: {current_angles}")
    else:
        print(f"Initial arm angles read successfully: {current_angles}")

    # Solve kinematic angles for the first CSV target waypoint
    x_first, y_first, z_first, g_first, duration_first = macro[0]
    first_target_angles = [
        math.degrees(a) for a in kinematics.get_robot_angles_radians(x_first, y_first, z_first)
    ] + [g_first]

    print(f"Gliding from current positions to starting waypoint (X:{x_first}, Y:{y_first}, Z:{z_first}) over {duration_first}s...")
    glide_to.glide_to(ser, current_angles, first_target_angles, duration_first)

    # Execute the rest of the macro segments
    for i in range(len(macro)-1):
        glide_to.glide_to_mm(ser, macro[i], macro[i+1])

