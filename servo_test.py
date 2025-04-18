import time

from adafruit_servokit import ServoKit
from kinematics import inverse_kinematics_2d

# Real Range test : 0 -> 135 deg 
# input Range test: 0 -> 180 deg
# translate 135 to true/real 135
angle = 0
TRUE_ANGLE = 135

# Set angle to true servo angle
# Min: 0
# Max: 135
def set_real_servo_angle(channel, angle):

    kit = ServoKit(channels=16)
    translated_angle = angle / TRUE_ANGLE * 180
    
    kit.servo[channel].angle = translated_angle

    print(f"Channel {channel} is set to: {angle}")

# Main
set_real_servo_angle(0, 110)
set_real_servo_angle(1, 110)
set_real_servo_angle(2, 110)

# 2D IK test
# theta1, theta2 = inverse_kinematics_2d(10, 5, L1=10, L2=10)

# set_real_servo_angle(0, theta1)
# set_real_servo_angle(1, theta2) 
