import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Setup I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pwm = PCA9685(i2c)
pwm.frequency = 50              

# Servo channels (change if yours are plugged in elsewhere)
COXA = 0
FEMUR = 1
TIBIA = 2

# Helper: Set angle for a servo channel
def set_angle(channel, angle):
    # Clamp angle between 0 and 180
    angle = max(0, min(180, angle))
    # Map to 16-bit duty cycle range
    duty = int((angle / 180.0) * 0.5 * 65535 + 0.05 * 65535)
    pwm.channels[channel].duty_cycle = duty

# Test sequence
def test_leg():
    print("Starting servo test...")
    
    # Neutral/default pose
    set_angle(COXA, 90)
    set_angle(FEMUR, 90)
    set_angle(TIBIA, 90)
    time.sleep(1)

    # Step forward
    set_angle(FEMUR, 60)  # Lift leg
    time.sleep(0.5)
    set_angle(COXA, 120)  # Move forward
    time.sleep(0.5)
    set_angle(FEMUR, 90)  # Lower leg
    time.sleep(0.5)

    # Reset to neutral
    set_angle(COXA, 90)
    time.sleep(0.5)

    print("Test complete.")

try:
    test_leg()
except KeyboardInterrupt:
    print("Exiting gracefully.")
finally:
    pwm.deinit()