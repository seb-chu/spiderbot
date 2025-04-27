import math
import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Setup I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pwm = PCA9685(i2c)
pwm.frequency = 50              

# Servo channels (change if yours are plugged in elsewhere)
COXA_CHANNEL = 0
FEMUR_CHANNEL = 1
TIBIA_CHANNEL = 2

# used for recalculating angles if overexceeded
FEMUR_CHANNELS = [1]
TIBIA_CHANNELS = [2]

# reverse polarity on specific servos
affected_channels = [2]

# ---------------- user-tunable femur limits --------------------
MAX_FEMUR_REL =  +45.0      # degrees, + = leg pitches downward
MIN_FEMUR_REL =  -75.0      # degrees, – = leg lifts upward
# ---------------------------------------------------------------

def set_servo_angle(channel, rel_angle, carry_angle=0.0):
    """
    channel      : PCA9685 channel number
    rel_angle    : ±90° convention (0 = straight & level)
    carry_angle  : spill passed from femur to tibia
    returns      : new spill (0 unless femur hit its limit)
    """

    # ---------- 1. femur clamp & spill -------------------------
    if channel in FEMUR_CHANNELS:
        spill = 0.0
        if rel_angle > MAX_FEMUR_REL:            # too far down
            spill     = rel_angle - MAX_FEMUR_REL
            rel_angle = MAX_FEMUR_REL
            print(f"[Femur] limited to +{MAX_FEMUR_REL}°, spill {spill:.1f}°")
        elif rel_angle < MIN_FEMUR_REL:          # too far up
            spill     = rel_angle - MIN_FEMUR_REL   # negative
            rel_angle = MIN_FEMUR_REL
            print(f"[Femur] limited to {MIN_FEMUR_REL}°, spill {spill:.1f}°")
        carry_angle = spill                      # hand to tibia

    elif channel in TIBIA_CHANNELS:
        rel_angle -= carry_angle                 # knee compensates
        carry_angle = 0.0                        # consumed

    # ---------- 2. map relative –90…+90  → absolute 0…180 -------
    abs_angle = rel_angle + 90.0                # keep in float
    abs_angle = max(0.0, min(180.0, abs_angle)) # final safety clamp

    # ---------- 3. PWM duty ------------------------------
    duty_min = 1500           # your board-specific values
    duty_max = 8250
    if channel in affected_channels:            # reversed servo
        duty_min, duty_max = duty_max, duty_min

    duty = int(duty_min + (abs_angle / 180.0) * (duty_max - duty_min))
    pwm.channels[channel].duty_cycle = duty

    print(f"Ch{channel}: rel {rel_angle:+6.1f}°, abs {abs_angle:6.1f}°, duty {duty}")
    return carry_angle


def set_all_servos_to_default(a):
    set_servo_angle(COXA_CHANNEL, a)
    extra_angle = set_servo_angle(FEMUR_CHANNEL, a)
    print("the extra angle is:", extra_angle)
    set_servo_angle(TIBIA_CHANNEL, a, extra_angle)


# simple increment from starting to ending angles, default increment is 1 deg
def increment_angle(channel, start_angle, end_angle, increment=1):
    """
    Move a servo smoothly from start_angle to end_angle.
    Handles +→+, –→–, +→–, –→+, any increment (int or float > 0).
    """
    increment = abs(increment)           # ensure positive magnitude
    if start_angle == end_angle:
        set_servo_angle(channel, start_angle)
        return

    step = math.copysign(increment, end_angle - start_angle)  # ±increment
    angle = start_angle

    while (end_angle - angle) * step >= 0:   # still on the same side of end?
        set_servo_angle(channel, angle)
        time.sleep(0.01)
        angle += step

    # Guarantee final exactly-on target (avoids overshoot/rounding issues)
    if angle != end_angle:
        set_servo_angle(channel, end_angle)



# TEST CODE
# RESET ALL TO 0

set_all_servos_to_default(0)

