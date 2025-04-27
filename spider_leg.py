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
# z_rest_helper = -2
# y_rest_helper = 5

# function to utilize IK to move joints accordingly
def kinematics(pos_x, pos_y, pos_z):
   
    # ==============
    # HELPER LINKS
    
    # Calculate Angle Positions
    h_link = math.sqrt(pos_x**2 + pos_y**2)
    l_helper = math.sqrt(pos_z**2 + (h_link - COXA_LINK)**2)
    
    # determine if link L is too large to physical structure
    if (l_helper > (FEMUR_LINK + TIBIA_LINK)):
        print(f"new total distance: {l_helper}")
        print(f"max possible length: {FEMUR_LINK + TIBIA_LINK}")
        print(f"ERROR: can't reach {pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f} position.")
        return
    # ==============

    # COXA JOINT
    coxa_joint_rad = math.atan2(pos_x, pos_y)
    
    # FEMUR JOINT
    print("lhelp", l_helper)
    print("numer", (FEMUR_LINK**2 + l_helper**2 - TIBIA_LINK**2))
    b_joint_helper_rad = math.acos((FEMUR_LINK**2 + l_helper**2 - TIBIA_LINK**2) / (2 * FEMUR_LINK * l_helper))
    
    # a_joint_helper_rad = math.atan(pos_z / (h_link - COXA_LINK)) # TODO same thing remove?
    a_joint_helper_rad = math.asin(pos_z / l_helper)
    
    femur_joint_rad = b_joint_helper_rad - a_joint_helper_rad


    # TIBIA JOINT
    tibia_joint_rad = math.acos((FEMUR_LINK**2 + TIBIA_LINK**2 - l_helper**2) / (2 * FEMUR_LINK * TIBIA_LINK))


    # RADIANS to DEGREE
    coxa_joint_deg = math.degrees(coxa_joint_rad)
    femur_joint_deg = math.degrees(femur_joint_rad)
    tibia_joint_deg = math.degrees(tibia_joint_rad)
    if pos_z < 0:                 # foot below the femur plane ➜ negative
        tibia_joint_deg = -tibia_joint_deg

    
    print(f'Angles (coxa, femur, tibia): {coxa_joint_deg:.2f}°, {femur_joint_deg:.2f}°, {tibia_joint_deg:.2f}°')
    # print(f'prev position (x, y, z): {pos_x:2f}, {y_rest_helper:2f}, {z_rest_helper:2f}')
    print(f'new position (x, y, z): {pos_x:2f}, {pos_y:2f}, {pos_z:2f}')
    print("-----")

    # Set servo angles
    # set_servo_angle(COXA_CHANNEL, coxa_joint_deg)
    # extra_angle = set_servo_angle(FEMUR_CHANNEL, femur_joint_deg)
    # if extra_angle > 0:
    #     print(f"EXTRA ANGLE: {extra_angle}")
    # set_servo_angle(TIBIA_CHANNEL, tibia_joint_deg, extra_angle)
    
   
    # Plot the leg in 3D
    # plot_leg(pos_x, pos_y, pos_z, tibia_joint_deg, femur_joint_deg)

# plot kinematics


# function to help translate x,y,z position with IK using interpolation (incrementation)


# move set servo angles with new angles (helper function)


# function to input x,y,z position and IK makes it move there


kinematics(0, 5, -3)
time.sleep(1)

# kinematics(0, 0.5, 0)
# time.sleep(1)

# kinematics(0, 0, 0)

# kinematics(0, -0.5, 0)
# time.sleep(3)
