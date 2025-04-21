import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

# Arm segment lengths
L1 = 70  # Coxa
L2 = 50  # Femur
L3 = 120 # Tibia

# Inverse kinematics (simplified for plotting)
def inverse_kinematics(x, y, z):
    theta_coxa = math.atan2(y, x)
    horizontal = math.sqrt(x**2 + y**2) - L1
    d = math.sqrt(horizontal**2 + z**2)
    
    # Triangle feasibility check
    if d > (L2 + L3): d = L2 + L3
    if d < abs(L2 - L3): d = abs(L2 - L3)
    
    theta_tibia = math.pi - math.acos((L2**2 + L3**2 - d**2) / (2 * L2 * L3))
    angle_a = math.atan2(z, horizontal)
    angle_b = math.acos((L2**2 + d**2 - L3**2) / (2 * L2 * d))
    theta_femur = angle_a + angle_b

    return theta_coxa, theta_femur, theta_tibia

# Forward kinematics to compute joint positions for plotting
def get_leg_points(theta_coxa, theta_femur, theta_tibia):
    # Base
    p0 = np.array([0, 0, 0])
    
    # Coxa endpoint
    p1 = np.array([
        L1 * math.cos(theta_coxa),
        L1 * math.sin(theta_coxa),
        0
    ])
    
    # Femur endpoint
    p2 = p1 + np.array([
        L2 * math.cos(theta_coxa) * math.cos(theta_femur),
        L2 * math.sin(theta_coxa) * math.cos(theta_femur),
        L2 * math.sin(theta_femur)
    ])
    
    # Tibia endpoint
    total_angle = theta_femur + theta_tibia
    p3 = p2 + np.array([
        L3 * math.cos(theta_coxa) * math.cos(total_angle),
        L3 * math.sin(theta_coxa) * math.cos(total_angle),
        L3 * math.sin(total_angle)
    ])
    
    return [p0, p1, p2, p3]

# Animate over time
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)
ax.set_zlim(-100, 150)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Generate circular motion of foot target
times = np.linspace(0, 2*np.pi, 100)
for t in times:
    x = 80 + 30 * math.cos(t)
    y = 80 + 30 * math.sin(t)
    z = 20 + 10 * math.sin(2*t)
    
    theta_coxa, theta_femur, theta_tibia = inverse_kinematics(x, y, z)
    points = get_leg_points(theta_coxa, theta_femur, theta_tibia)
    
    xs, ys, zs = zip(*points)
    
    ax.cla()
    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    ax.set_zlim(-100, 150)
    ax.plot(xs, ys, zs, marker='o', linewidth=3, markersize=5)
    ax.scatter(x, y, z, color='red')  # target point
    plt.pause(0.05)

plt.show()
