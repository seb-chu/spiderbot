import math
from set_servo_angles import *
# INITIALIZATION

# Servo angles (deg)
# servo_angle_coxa, servo_angle_femur, servo_angle_tibia = 0, 30, 30

# Link Lengths (inches)
COXA_LINK = 2.0
FEMUR_LINK = 2.5
TIBIA_LINK = 5

# Servo channels (change if yours are plugged in elsewhere)
COXA_CHANNEL = 0
FEMUR_CHANNEL = 1
TIBIA_CHANNEL = 2

COXA_ZERO_DEG = 90
FEMUR_ZERO_DEG = 90
TIBIA_ZERO_DEG = 90

# Helper Variables (inches)
z_rest_helper = 3.0
y_rest_helper = 4.0

# function to utilize IK to move joints accordingly
def kinematics(pos_x, pos_y, pos_z):
    
    # adds offset to new position
    pos_z += z_rest_helper

    pos_y = -pos_y # flipping y value to correct y_position
    pos_y += y_rest_helper # TODO do i need coxa_link offset for y? to get the exact tip_point
    # -1 + 4 = 3
    # 1 + 4 = 5
    # Calculate Angle Positions
    l_helper = math.sqrt(pos_z**2 + pos_y**2)
    
    if (l_helper + COXA_LINK > (FEMUR_LINK + TIBIA_LINK)):
        print(f"new pos length: {l_helper + COXA_LINK}")
        print(f"max possible length: {FEMUR_LINK + TIBIA_LINK}")
        print(f"ERROR: can't reach {pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f} position.")
        return 
    
    tibia_joint_rad = math.acos((FEMUR_LINK**2 + TIBIA_LINK**2 - l_helper**2) / (2 * FEMUR_LINK * TIBIA_LINK))

    b_joint_helper_rad = math.acos((l_helper**2 + FEMUR_LINK**2 - TIBIA_LINK**2) / (2 * l_helper * FEMUR_LINK))

    a_joint_helper_rad = math.atan(pos_z / pos_y)

    femur_joint_rad = b_joint_helper_rad - a_joint_helper_rad

    tibia_joint_deg = tibia_joint_rad * (180 / math.pi)
    femur_joint_deg = femur_joint_rad * (180 / math.pi)
    
    print(f'Angles (coxa, femur, tibia): {0:.2f}°, {femur_joint_deg:.2f}°, {tibia_joint_deg:.2f}°')
    print(f'prev position (x, y, z): {pos_x:2f}, {y_rest_helper:2f}, {z_rest_helper:2f}')
    print(f'new position (x, y, z): {pos_x:2f}, {pos_y:2f}, {pos_z:2f}')

    # Set servo angles
    set_relative_servo_angle(COXA_CHANNEL, 0)
    set_relative_servo_angle(FEMUR_CHANNEL, femur_joint_deg)
    set_relative_servo_angle(TIBIA_CHANNEL, tibia_joint_deg)
    
   

    # Plot the leg in 3D
    # plot_leg(pos_x, pos_y, pos_z, tibia_joint_deg, femur_joint_deg)

# plot kinematics


# function to help translate x,y,z position with IK using interpolation (incrementation)


# move set servo angles with new angles (helper function)


# function to input x,y,z position and IK makes it move there

kinematics(0, 0, 0)
