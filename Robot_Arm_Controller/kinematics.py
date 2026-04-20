import math

top_arm = 64
bottom_arm = 100

def base_rotation (x: int | float, y: int | float):

   radian_base_angle = math.atan2(y,x)

   if radian_base_angle < 0: radian_base_angle += math.pi

   return radian_base_angle
   

def top_kinematics (x: int | float, y: int | float, z: int | float):
   new_x_y = math.sqrt(x ** 2 + y **2)
   if new_x_y == 0: new_x_y = 1


   base = math.sqrt(z**2 + new_x_y ** 2)

   top_angle = math.acos((top_arm ** 2 + bottom_arm ** 2 - base ** 2) / (2 * top_arm * bottom_arm))

   bottom_angle = math.asin((top_arm * math.sin(top_angle)) / base) + math.atan(z/new_x_y)

   return math.degrees(top_angle) - 45, math.degrees(bottom_angle)

# while True:
#    x =int(input("x :"))
#    y =int(input("y :"))
#    z =int(input("z :"))
#    top_angles = top_kinematics(x, y, z)
#    servoOneInput = int((top_angles[0]))
#    servoTwoInput = int((base_rotation(x, y)))
#    servoThreeInput = int((top_angles[1]))
#    print(servoOneInput,servoTwoInput,servoThreeInput)