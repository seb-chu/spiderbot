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

COXA_ZERO_DEG = 90
FEMUR_ZERO_DEG = 90
TIBIA_ZERO_DEG = 90

# Helper: Set angle for a servo channel
def set_servo_angle(channel, angle):
    angle = max(0, min(180, angle))  # Clamp angle
    min_duty = 1500
    max_duty = 8250
    duty = int(min_duty + (angle / 180.0) * (max_duty - min_duty))
    pwm.channels[channel].duty_cycle = duty
    print(f"Angle: {angle}°, Duty: {duty}")

# real relative
def set_relative_servo_angle(channel, angle):
    set_servo_angle(channel, 90 + angle)

def set_all_servos_to_default():
    set_relative_servo_angle(COXA_CHANNEL, 0)
    set_relative_servo_angle(FEMUR_CHANNEL, 0)
    set_relative_servo_angle(TIBIA_CHANNEL, 0)
# TEST CODE
# RESET ALL TO 0

# SAME LOGIC
# set_servo_angle(TIBIA_CHANNEL, 90)
# set_relative_servo_angle(TIBIA_CHANNEL, 0)