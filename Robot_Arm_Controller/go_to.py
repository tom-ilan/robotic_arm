import serial
import time
from num_packer import num_packer
import math
import kinematics

# Function to move the robot arm to a specific x, y, z coordinate in millimeters
# It is precise due to being able to drive in centi degrees, but it has less ROM, of only 45-135 degrees.
def go_to_precise(ser, x_mm: float, y_mm: float, z_mm: float):

        # get_robot_angles_degrees returns radians — convert to degrees before packing
        base_rad, bottom_rad, top_rad = kinematics.get_robot_angles_degrees(x_mm, y_mm, z_mm)
        base_degrees = math.degrees(base_rad)
        bottom_degrees = math.degrees(bottom_rad)
        top_degrees = math.degrees(top_rad)

        # Packs angles into bytes for sending to arduino
        base_degrees_packed_big, base_degrees_packed_little = num_packer(base_degrees)
        bottom_degrees_packed_big, bottom_degrees_packed_little = num_packer(bottom_degrees)
        top_degrees_packed_big, top_degrees_packed_little = num_packer(top_degrees)


        ser.write(bytes([base_degrees_packed_big, base_degrees_packed_little, bottom_degrees_packed_big, bottom_degrees_packed_little, top_degrees_packed_big, top_degrees_packed_little]))
        ser.flush()
        time.sleep(0.01)