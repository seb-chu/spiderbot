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

# Helper Variables (inches)
# z_rest_helper = -2
# y_rest_helper = 5

# function to utilize IK to move joints accordingly
def kinematics(pos_x, pos_y, pos_z):
   
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
    set_servo_angle(COXA_CHANNEL, coxa_deg)
    set_servo_angle(FEMUR_CHANNEL, femur_deg)
    set_servo_angle(TIBIA_CHANNEL, tibia_deg)

# plot kinematics


# function to help translate x,y,z position with IK using interpolation (incrementation)


# move set servo angles with new angles (helper function)


# function to input x,y,z position and IK makes it move there


# kinematics(0, FEMUR_LINK + TIBIA_LINK + COXA_LINK, 0)
# time.sleep(2)



kinematics(0, 7, -2)
time.sleep(2)

kinematics(0, 8, -1)
time.sleep(2)

kinematics(2, 6, -2)
time.sleep(2)



kinematics(0, 7, -2)
time.sleep(2)

kinematics(0, 7, -2)
time.sleep(2)