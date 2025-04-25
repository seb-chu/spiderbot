import math
from adafruit_servokit import ServoKit


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

