import math

def inverse_kinematics_2d(x, y, L1, L2):
    distance = math.hypot(x, y)
    
    if distance > (L1 + L2):
        raise ValueError("Target is unreachable")

    # Compute theta2
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_theta2)  # in radians

    # Compute theta1
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    # Convert to degrees
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)

    return theta1_deg, theta2_deg
