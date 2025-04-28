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

# used for recalculating angles if overexceeded
FEMUR_CHANNELS = [1]
TIBIA_CHANNELS = [2]

# reverse polarity on specific servos
affected_channels = [BACK_RIGHT_TIBIA_CHANNEL,
                     FRONT_RIGHT_TIBIA_CHANNEL,
                     FRONT_LEFT_COXA_CHANNEL,
                     FRONT_LEFT_TIBIA_CHANNEL,
                     BACK_LEFT_COXA_CHANNEL,
                     BACK_LEFT_TIBIA_CHANNEL]

# ---------------- user-tunable femur limits --------------------
MAX_FEMUR_REL =  +45.0      # degrees, + = leg pitches downward
MIN_FEMUR_REL =  -75.0      # degrees, – = leg lifts upward
# ---------------------------------------------------------------


# map leg-id → (coxa-ch, femur-ch, tibia-ch)
LEG_CHANNELS = {
    0: (FRONT_RIGHT_COXA_CHANNEL, FRONT_RIGHT_FEMUR_CHANNEL, FRONT_RIGHT_TIBIA_CHANNEL),
    1: (FRONT_LEFT_COXA_CHANNEL,  FRONT_LEFT_FEMUR_CHANNEL,  FRONT_LEFT_TIBIA_CHANNEL),
    2: (BACK_RIGHT_COXA_CHANNEL,  BACK_RIGHT_FEMUR_CHANNEL,  BACK_RIGHT_TIBIA_CHANNEL),
    3: (BACK_LEFT_COXA_CHANNEL,   BACK_LEFT_FEMUR_CHANNEL,   BACK_LEFT_TIBIA_CHANNEL),
}

def set_leg_angles(leg_id: int, angles):
    """
    angles : (coxa_deg, femur_deg, tibia_deg) in your ±90° convention
    Handles femur-spill → tibia compensation automatically.
    """
    coxa_deg, femur_deg, tibia_deg = angles
    coxa_ch, femur_ch, tibia_ch    = LEG_CHANNELS[leg_id]

    # femur may return a spill that knee must absorb
    spill = set_servo_angle(femur_ch, femur_deg)
    set_servo_angle(coxa_ch,  coxa_deg)        # coxa unaffected
    set_servo_angle(tibia_ch, tibia_deg, spill)


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
    abs_angle = rel_angle + 90.0                # mapping to 0–180°
    
    # ---------- Adjust for "down is negative" (inverted) ---------
    abs_angle = max(0.0, min(180.0, abs_angle)) # safety clamp

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


# TEST CODE
# RESET ALL TO 0

# set_all_servos_to_default(0)
