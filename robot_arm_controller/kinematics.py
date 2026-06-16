import math

# ── PHYSICAL CONFIGURATION ───────────────────────────────────────────────────
# Easily adjust the mechanical arm segment lengths here:
BOTTOM_ARM_LENGTH_MM = 100.0   # Shoulder-to-Elbow length (Shoulder)
TOP_ARM_LENGTH_MM    = 64.0    # Elbow-to-End-Effector length (Elbow)

TOP_MOUNTING_OFFSET_ANGLE_RADIANS = -math.pi / 2

# Compute the robot arm angles,
# given a target x, y and z.
def get_robot_angles_radians(x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float]:
   
   # Compute base angle.
   base_angle_radians = math.atan2(y_mm, x_mm)

   # If the base angle is less than 0,
   # we cannot move the servo to that position.
   # Instead, we move the arm to the opposite position,
   # And invert the target later.
   inverted_x = base_angle_radians < 0
   if inverted_x: 
      base_angle_radians += math.pi

   # Transform x, y, and z coordinates to post-base angle positioning,
   # where the arm is in line with the target.
   in_line_x_mm = math.sqrt(x_mm ** 2 + y_mm ** 2) * (-1 if inverted_x else 1)
   in_line_z_mm = z_mm

   # Get distance and angle to target.
   # Angle relative to x axis.
   target_distance_mm = math.sqrt(in_line_z_mm ** 2 + in_line_x_mm ** 2)
   target_angle_radians = math.atan2(in_line_z_mm, in_line_x_mm)

   # Apply cosine law to find top angle.
   top_angle_radians = math.acos(
      (TOP_ARM_LENGTH_MM ** 2 + BOTTOM_ARM_LENGTH_MM ** 2 - target_distance_mm ** 2) / 
      (2 * TOP_ARM_LENGTH_MM * BOTTOM_ARM_LENGTH_MM)
   )
   
   # Apply sine law to find bottom angle.
   target_to_bottom_angle_radians = math.asin(
      (TOP_ARM_LENGTH_MM * math.sin(top_angle_radians)) /
      target_distance_mm
   )
   
   # Apply offset from target.
   bottom_angle_radians = target_angle_radians + target_to_bottom_angle_radians

   # Apply offset to top angle.
   top_angle_radians += TOP_MOUNTING_OFFSET_ANGLE_RADIANS

   # Make sure all angles are positive.
   assert math.pi >= base_angle_radians >= 0
   assert math.pi >= bottom_angle_radians >= 0
   assert math.pi >= top_angle_radians >= 0

   return base_angle_radians, bottom_angle_radians, top_angle_radians
