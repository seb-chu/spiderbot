import math
# from set_angles import set_real_servo_angle

# INITIALIZATION
# Servo angles (deg)
servo_angle_coxa, servo_angle_femur, servo_angle_tibia = 0, 30, 30

# Link Lengths (inches)
COXA_LINK = 1.0
FEMUR_LINK = 1.0
TIBIA_LINK = 1.0

# Helper Variables
z_rest_helper = 0.3
y_rest_helper = 0.4

# function to utilize IK to move joints accordingly
def kinematics(pos_x, pos_y, pos_z):
    
    # adds offset to new position
    pos_z += z_rest_helper
    pos_y += y_rest_helper # TODO do i need coxa_link offset for y? to get the exact tip_point

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
    # set_real_servo_angle(0, 0) # Coxa
    # set_real_servo_angle(1, femur_joint_deg) # Femur
    # set_real_servo_angle(2, tibia_joint_deg) # Tibia
    
   

    # Plot the leg in 3D
    # plot_leg(pos_x, pos_y, pos_z, tibia_joint_deg, femur_joint_deg)

# plot kinematics


# function to help translate x,y,z position with IK using interpolation (incrementation)


# move set servo angles with new angles (helper function)


# function to input x,y,z position and IK makes it move there

kinematics(0, 0, 0)