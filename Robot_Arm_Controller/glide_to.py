import time
import math
import kinematics
import go_to

def glide (current_angle, target_angle, glide_time):
    # Calculate the difference in angles
    angle_diff = target_angle - current_angle
    
    # Calculate the number of steps based on the time and a fixed step duration
    global step_duration
    step_duration = 0.5
    steps = max(1, int(glide_time / step_duration))
    
    # Calculate the angle increment for each step
    angle_increment = angle_diff / steps
    
    # Create a list to hold the angles for each step
    angles = []
    
    for i in range(steps):
        current_angle += angle_increment
        angles.append(current_angle)
    
    return angles

def glide_to(ser, current_angles, target_angles, glide_time):
    base_current, bottom_current, top_current, gripper_current = current_angles
    base_target, bottom_target, top_target, gripper_target = target_angles

    base_angles = glide(base_current, base_target, glide_time)
    bottom_angles = glide(bottom_current, bottom_target, glide_time)
    top_angles = glide(top_current, top_target, glide_time)
    gripper_angles = glide(gripper_current, gripper_target, glide_time)

    for i in range(len(base_angles)):
        go_to.go_to_angle(ser, base_angles[i], bottom_angles[i], top_angles[i], gripper_angles[i])
        time.sleep(step_duration)

def glide_to_mm(ser, current_xyz_g, target_xyz_g):
    """
    Accepts current and target states as [x, y, z, gripper, time] lists.
    Calculates kinematics internally and performs the joint glide motion.
    """
    x_c, y_c, z_c, g_c, _ = current_xyz_g
    x_t, y_t, z_t, g_t, duration = target_xyz_g

    c_base, c_bot, c_top = [
        math.degrees(a) for a in kinematics.get_robot_angles_radians(x_c, y_c, z_c)
    ]
    t_base, t_bot, t_top = [
        math.degrees(a) for a in kinematics.get_robot_angles_radians(x_t, y_t, z_t)
    ]

    glide_to(
        ser,
        (c_base, c_bot, c_top, g_c),
        (t_base, t_bot, t_top, g_t),
        duration
    )