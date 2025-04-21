import board
import busio
import math
from adafruit_pca9685 import PCA9685

# Setup I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pwm = PCA9685(i2c)
pwm.frequency = 50              


def inverse_kinematics(x, y, z, L1=70, L2=50, L3=120): #target coordinates from hip, length of coxa, femur, tibia
    theta_coxa = math.atan2(y, x)
    horizontal = math.sqrt(x**2 + y**2) - L1
    d = math.sqrt(horizontal**2 + z**2)
    
    if d > (L2 + L3):
        raise ValueError("Target out of reach")

    theta_tibia = math.pi - math.acos((L2**2 + L3**2 - d**2) / (2 * L2 * L3))
    angle_a = math.atan2(z, horizontal)
    angle_b = math.acos((L2**2 + d**2 - L3**2) / (2 * L2 * d))
    theta_femur = angle_a + angle_b

    return (
        math.degrees(theta_coxa),
        math.degrees(theta_femur),
        math.degrees(theta_tibia)
    )

def set_angle(channel, angle):
    angle = max(0, min(180, angle))  # Clamp angle
    min_duty = 1500
    max_duty = 8250
    duty = int(min_duty + (angle / 180.0) * (max_duty - min_duty))
    pwm.channels[channel].duty_cycle = duty
    print(f"Angle: {angle}°, Duty: {duty}")

coxa_deg, femur_deg, tibia_deg = inverse_kinematics(20,20,20)

set_angle(0, coxa_deg)
set_angle(1, femur_deg)
set_angle(2, tibia_deg)
