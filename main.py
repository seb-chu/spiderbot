import math
from set_servo_angles import *
import time
from kinematics import kinematics
from walk_cycle import walk_cycle

dt = 0.02
gait = walk_cycle(dt=dt, stride=2.0, lift=1.0)

for pose in gait:             # pose is {leg_id:(x,y,z)}
    for leg,(x,y,z) in pose.items():
        angles = kinematics(x, y, z, leg)     # your IK returns tuple
        set_leg_angles(leg, angles)           # drive servos
    time.sleep(dt)
