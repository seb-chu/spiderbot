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

COXA_ZERO_DEG = 90
FEMUR_ZERO_DEG = 90
TIBIA_ZERO_DEG = 90

MAX_FEMUR_ANGLE = 125



# reverse polarity on specific servos
affected_channels = [2]

# Helper: Set angle for a servo channel
def set_servo_angle(channel, rel_angle, carry_angle=0.0):
    """
    rel_angle : desired servo position in degrees, –90 (left/up) … +90 (right/down)
    carry_angle : surplus degrees from femur that should bend the tibia

    Returns:
        updated carry_angle (0 if nothing spilled over)
    """
    # 1. map relative –90 … +90  →  absolute 0 … 180
    abs_angle = rel_angle + 90.0

    # 2. optional femur-tibia spill-over
    if channel in FEMUR_CHANNELS:
        if abs_angle > MAX_FEMUR_ANGLE:
            carry_angle = abs_angle - MAX_FEMUR_ANGLE
            abs_angle   = MAX_FEMUR_ANGLE
            print(f"[Femur] limited to {MAX_FEMUR_ANGLE} abs° or {MAX_FEMUR_ANGLE-90} rel°, spill {carry_angle:.1f}°")
        else:
            carry_angle = 0.0            # nothing to pass along

    elif channel in TIBIA_CHANNELS:
        abs_angle -= carry_angle         # add the spill (really subtract because abs)
        abs_angle = max(0, min(180, abs_angle))  # safety clamp

    # 3. convert 0 … 180° to 16-bit duty (1 ms … 2 ms pulse at 50 Hz)
    #    1 ms / 20 ms = 0.05  →  0.05 * 65 535 ≈ 3277
    #    2 ms / 20 ms = 0.10  →  0.10 * 65 535 ≈ 6554
    duty_min = 1500
    duty_max = 8250
    if channel in affected_channels:     # reverse-rotation servos
        duty_min, duty_max = duty_max, duty_min

    duty = int(duty_min + (abs_angle / 180.0) * (duty_max - duty_min))
    pwm.channels[channel].duty_cycle = duty

    print(f"Ch {channel}: rel {abs_angle-90:+6.1f}°, abs {abs_angle:6.1f}°, duty {duty}")

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

# set_all_servos_to_default(0)