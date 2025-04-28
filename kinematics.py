import math
from set_servo_angles import *
import time
# INITIALIZATION

# Servo angles (deg)
# servo_angle_coxa, servo_angle_femur, servo_angle_tibia = 0, 30, 30

# Link Lengths (inches)
COXA_LINK = 2.0
FEMUR_LINK = 2.5
TIBIA_LINK = 5

# Servo channels (change if yours are plugged in elsewhere)
FRONT_RIGHT_COXA_CHANNEL = 0
FRONT_RIGHT_FEMUR_CHANNEL = 1
FRONT_RIGHT_TIBIA_CHANNEL = 2

FRONT_LEFT_COXA_CHANNEL = 12
FRONT_LEFT_FEMUR_CHANNEL = 13
FRONT_LEFT_TIBIA_CHANNEL = 14

BACK_RIGHT_COXA_CHANNEL = 4
BACK_RIGHT_FEMUR_CHANNEL = 5
BACK_RIGHT_TIBIA_CHANNEL = 6

BACK_LEFT_COXA_CHANNEL = 8
BACK_LEFT_FEMUR_CHANNEL = 9
BACK_LEFT_TIBIA_CHANNEL = 10

# Leg Type
FRONT_RIGHT_LEG = 0
FRONT_LEFT_LEG = 1
BACK_RIGHT_LEG = 2
BACK_LEFT_LEG = 3

# function to utilize IK to move joints accordingly
def kinematics(pos_x, pos_y, pos_z, leg_type = 0):
    
    # ------------- Select Leg Type -------------
    coxa_channel = 0
    femur_channel = 0
    tibia_channel = 0

    match leg_type:
        # FR
        case 0:
            coxa_channel = FRONT_RIGHT_COXA_CHANNEL
            femur_channel = FRONT_RIGHT_FEMUR_CHANNEL
            tibia_channel = FRONT_RIGHT_TIBIA_CHANNEL
        # FL
        case 1:
            coxa_channel = FRONT_LEFT_COXA_CHANNEL
            femur_channel = FRONT_LEFT_FEMUR_CHANNEL
            tibia_channel = FRONT_LEFT_TIBIA_CHANNEL
        # BR
        case 2:
            coxa_channel = BACK_RIGHT_COXA_CHANNEL
            femur_channel = BACK_RIGHT_FEMUR_CHANNEL
            tibia_channel = BACK_RIGHT_TIBIA_CHANNEL
        # BL
        case 3:
            coxa_channel = BACK_LEFT_COXA_CHANNEL
            femur_channel = BACK_LEFT_FEMUR_CHANNEL
            tibia_channel = BACK_LEFT_TIBIA_CHANNEL

    # ------------- helper lengths -------------
    h = math.hypot(pos_x, pos_y)                       # ground-plane distance
    l = math.hypot(pos_z, h - COXA_LINK)           # ankle-to-hip distance

    reach = FEMUR_LINK + TIBIA_LINK
    if l > reach:
        print(f"Target (%.2f, %.2f, %.2f) out of reach (%.2f > %.2f)" %
              (pos_x, pos_y, pos_z, l, reach))
        return

    # ------------- COXA (yaw) -----------------
    coxa_deg = math.degrees(math.atan2(pos_x, pos_y))          # –180 … +180

    # ------------- FEMUR (pitch) ---------------
    # hip elevation angle: signed
    hip_elev = math.atan2(-pos_z, h - COXA_LINK)           # up = +ve
    # law-of-cosines between femur & tibia
    cos_b = (FEMUR_LINK**2 + l**2 - TIBIA_LINK**2) / (2 * FEMUR_LINK * l)
    cos_b = max(-1.0, min(1.0, cos_b))
    b = math.acos(cos_b)

    femur_deg = math.degrees(hip_elev + b)      # 0° = straight

    # ------------- TIBIA (knee) ----------------
    cos_k = (FEMUR_LINK**2 + TIBIA_LINK**2 - l**2) / (2 * FEMUR_LINK * TIBIA_LINK)
    cos_k = max(-1.0, min(1.0, cos_k))
    inner_knee = math.degrees(math.acos(cos_k))        # 0…180
    tibia_deg = inner_knee                    # 0° = straight
    # Flexion down is negative if you want:
    tibia_deg -= 180  

    print(f"Angles  (coxa, femur, tibia) : {coxa_deg:+.2f}°, {femur_deg:+.2f}°, {tibia_deg:+.2f}°")
    print(f"Position (pos_x, pos_y, pos_z)           : {pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f}\n")

    # Set servo angles
    set_servo_angle(coxa_channel, coxa_deg)
    set_servo_angle(femur_channel, femur_deg)
    set_servo_angle(tibia_channel, tibia_deg)
    time.sleep(0.5)

# function to input x,y,z position and IK makes it move there

# WALK FORWARD CYCLE
def forward(leg_type):
    kinematics(0.00, 6.92, 2.66, leg_type) # resting position

    kinematics(0.00, 8.58, 0.68, leg_type) # up 

    kinematics(-4.29, 7.43, 0.68, leg_type) # forward (up)
    kinematics(-3.46, 5.99, 2.66, leg_type) # down

    kinematics(0.00, 6.92, 2.66, leg_type) # resting position


# TEST SERVOS
# test_num = [0.0, -20.0, 0.0, 20, 0.0]

# for a in test_num:
#     # [MOVES CORRECTLY]
#     # set_servo_angle(FRONT_RIGHT_COXA_CHANNEL, a)
#     # set_servo_angle(FRONT_RIGHT_FEMUR_CHANNEL, a)
#     # set_servo_angle(FRONT_RIGHT_TIBIA_CHANNEL, a)

#     # [MOVES CORRECTLY]
#     # set_servo_angle(BACK_RIGHT_COXA_CHANNEL, a)
#     # set_servo_angle(BACK_RIGHT_FEMUR_CHANNEL, a)
#     # set_servo_angle(BACK_RIGHT_TIBIA_CHANNEL, a)  

#     # [MOVES CORRECTLY]
#     # set_servo_angle(FRONT_LEFT_COXA_CHANNEL, a)
#     # set_servo_angle(FRONT_LEFT_FEMUR_CHANNEL, a)
#     # set_servo_angle(FRONT_LEFT_TIBIA_CHANNEL, a)

#     # []
#     # set_servo_angle(BACK_LEFT_COXA_CHANNEL, a)
#     # set_servo_angle(BACK_LEFT_FEMUR_CHANNEL, a)
#     set_servo_angle(BACK_LEFT_TIBIA_CHANNEL, a)
#     time.sleep(0.5)

# forward(FRONT_RIGHT_LEG) # backwards, add negative to kine(x,...)
# forward(FRONT_LEFT_LEG) # backwards,
# forward(BACK_RIGHT_LEG) # backwards, leg is lower than others
# forward(BACK_LEFT_LEG) # backwards, leg is lower than others

# FUNCTIONS FOR SIMPLE MOVEMENT
def zero_position(leg_type):
    kinematics(0, FEMUR_LINK + TIBIA_LINK + COXA_LINK, 0, leg_type)
    
# Set starting position for leg
def standard_position(leg_type):
    if leg_type in [BACK_RIGHT_LEG, BACK_LEFT_LEG]:
        kinematics(-3.46, 5.99, 2.66, leg_type)
    else:
        kinematics(3.46, 5.99, 2.66, leg_type)

# zero_position(FRONT_RIGHT_LEG)
# time.sleep(2)
standard_position(FRONT_RIGHT_LEG)
standard_position(BACK_RIGHT_LEG)
standard_position(FRONT_LEFT_LEG)
standard_position(BACK_LEFT_LEG)
#kinematics(0.00, 6.92, 2.66, BACK_LEFT_LEG)

# FORWARD LOGIC
# - move each leg forward before reset to default positoin
# - once we reach the last leg, once it moves forward, move all legs back to standing position
kinematics(3.46, 5.99, 2.66, BACK_LEFT_LEG) # down
