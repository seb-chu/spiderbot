import math
from set_servo_angles import *
import time
from kinematics import *

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
capture_stance()      # call once (feet at HOME_FOOT)

move_body(+1.0)       # body 1 inch forward
time.sleep(0.2)


move_body(-1.0)       # slide back half-inch