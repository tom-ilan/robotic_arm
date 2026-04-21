import math

top_arm = 64
bottom_arm = 100

def base_rotation (x: int | float, y: int | float):
   base_angle = math.atan2(y,x)

   if base_angle < 0: base_angle += math.pi

   return math.degrees(base_angle)
   

def top_kinematics (x: int | float, y: int | float, z: int | float):
   new_x_y = math.sqrt(x ** 2 + y **2) # Acounts for base rotation

   if new_x_y == 0: new_x_y = 1 # For avoiding division by zero error 

   base = math.sqrt(z**2 + new_x_y ** 2) # Hypotenous of the triangle formed by new_x_y and z

   top_angle = math.acos((top_arm ** 2 + bottom_arm ** 2 - base ** 2) / (2 * top_arm * bottom_arm)) # Using cosine law

   bottom_angle = math.asin((top_arm * math.sin(top_angle)) / base) + math.atan(z/new_x_y) # Using sine law

   return math.degrees(top_angle) - 45, math.degrees(bottom_angle)