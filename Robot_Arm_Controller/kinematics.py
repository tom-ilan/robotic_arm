# For kinemtaic calculations
from math import sin, cos, sqrt, acos, atan, degrees

# Length of each arm
lower_arm_length = 100
upper_arm_length = 64

# Kinematics for the arm 
def arm_x_y_kinematics(x: int | float, y: int | float):

   # Corrects division by zero
   if x == 0: x = 1
   if y == 0: y = 1

   # Checks if the input is a valid point | float
   if sqrt(x ** 2 + y ** 2) > 164 or sqrt(x ** 2 + y ** 2) < 36:
      return 'Not a valid point | float'
     
   else:
      # Angle calculations as described by:
      # x = L1 * cos(θ1) + L2 * cos (θ1 + θ2)
      # y = L1 * sin(θ1) + L2 * sin (θ1 + θ2)
      upper_arm_angle_rad = acos((x**2 + y**2 - lower_arm_length**2 - upper_arm_length**2) / (2 * lower_arm_length * upper_arm_length))
      lower_arm_angle_rad = (atan(y / x) - atan(upper_arm_length * sin(upper_arm_angle_rad) / (lower_arm_length + upper_arm_length * cos(upper_arm_angle_rad))))
      return (degrees(upper_arm_angle_rad), degrees(lower_arm_angle_rad))

# Kinematics for the base
def arm_z_x_kinematics(x: int | float, z: int | float):
    
   # Corrects division by zero
   if x == 0: x = 1
   if z == 0: y = 1

    # Checks if the input is a valid point | float
   if sqrt(x ** 2 + z ** 2) > 164 or sqrt(x ** 2 + z ** 2) < 36:
      return 'Not a valid point | float'
   
   else:
      # Calculates the base angle
      base_angle = atan(z/x)
      return base_angle

# Does the kinematics for the whole arm.
def arm_full_kinematics(x: int | float, y: int | float, z:  int | float):
   
   # Calculates base angles
   base_angle = arm_z_x_kinematics(x, z)

   # Calculates arm angles
   arm_x = sqrt(z ** 2 + x ** 2)
   upper_arm_angle = arm_x_y_kinematics(arm_x, y)[0]
   lower_arm_angle = 180 - arm_x_y_kinematics(arm_x, y)[1]

   if lower_arm_angle < 180 and upper_arm_angle < 180 and base_angle < 180:
      return lower_arm_angle, upper_arm_angle, base_angle
   
   else: return 'Not a valid point'

print(arm_full_kinematics(123,100,0))