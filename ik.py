import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

# Arm segment lengths
# L1 = 70  # Coxa
# L2 = 50  # Femur
# L3 = 120 # Tibia

L1 = 1  # Coxa
L2 = 1  # Femur
L3 = 1 # Tibia

# Inverse kinematics (simplified for plotting)
def inverse_kinematics(x, y, z):
    # Calculate the coxa angle (angle with the x-axis)
    theta_coxa = math.atan2(y, x)

    # Calculate the horizontal distance in the XY plane, subtract L1 for Coxa
    horizontal = math.sqrt(x**2 + y**2) - L1

    # Calculate the distance from the base to the target position
    d = math.sqrt(horizontal**2 + z**2)

    # Feasibility check for the triangle (between femur and tibia)
    if d > (L2 + L3): d = L2 + L3
    if d < abs(L2 - L3): d = abs(L2 - L3)

    # Calculate the angle for the tibia (between femur and tibia)
    theta_tibia = math.pi - math.acos((L2**2 + L3**2 - d**2) / (2 * L2 * L3))

    # Calculate the angle for the femur using two components
    angle_a = math.atan2(z, horizontal)  # Vertical angle relative to the horizontal plane
    angle_b = math.acos((L2**2 + d**2 - L3**2) / (2 * L2 * d))  # Angle to form the correct triangle
    theta_femur = angle_a + angle_b

    return theta_coxa, theta_femur, theta_tibia


# Function to calculate joint angles for a new position
def calculate_joint_angles(new_x, new_y, new_z):
    # Ensure the new position is reachable within the leg's range
    reachable_distance = L1 + L2 + L3  # Maximum reach of the leg
    target_distance = math.sqrt(new_x**2 + new_y**2 + new_z**2)
    
    if target_distance > reachable_distance:
        print(f"Target position ({new_x}, {new_y}, {new_z}) is out of reach!")
        return None
    
    # Use inverse kinematics to calculate joint angles
    theta_coxa, theta_femur, theta_tibia = inverse_kinematics(new_x, new_y, new_z)
    
    # Return the calculated joint angles
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
    total_angle = theta_femur + theta_tibia  # Add femur and tibia angles
    p3 = p2 + np.array([
        L3 * math.cos(theta_coxa) * math.cos(total_angle),
        L3 * math.sin(theta_coxa) * math.cos(total_angle),
        L3 * math.sin(total_angle)
    ])
    
    return [p0, p1, p2, p3]

# def display_simulation():
#     # Animate over time
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.set_xlim(-200, 200)
#     ax.set_ylim(-200, 200)
#     ax.set_zlim(-100, 150)
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')

#     # Generate circular motion of foot target
#     times = np.linspace(0, 2*np.pi, 100)
#     for t in times:
#         x = 80 + 30 * math.cos(t)
#         y = 80 + 30 * math.sin(t)
#         z = 20 + 10 * math.sin(2*t)
        
#         theta_coxa, theta_femur, theta_tibia = inverse_kinematics(x, y, z)
#         points = get_leg_points(theta_coxa, theta_femur, theta_tibia)
        
#         # DEBUGGING STATEMENTS
#         print(f"(x,y,z): {x:.2f}, {y:.2f}, {z:.2f}")
#         print(f"(Coxa, Femur, Tibia): {math.degrees(theta_coxa):.2f}°, {math.degrees(theta_femur):.2f}°, {math.degrees(theta_tibia):.2f}°")
#         print("")

#         xs, ys, zs = zip(*points)
        
#         ax.cla()
#         ax.set_xlim(-200, 200)
#         ax.set_ylim(-200, 200)
#         ax.set_zlim(-100, 150)
#         ax.plot(xs, ys, zs, marker='o', linewidth=3, markersize=5)
#         ax.scatter(x, y, z, color='red')  # target point
#         plt.pause(0.05)

#     plt.show()

def move_and_calculate_positions(x, y, z, increment, axis='x'):
    # Increment the position based on the axis
    if axis == 'x':
        x += increment
    elif axis == 'y':
        y += increment
    elif axis == 'z':
        z += increment
    
    # Ensure the new position is reachable within the leg's range
    LEG_SUM = abs(L1) + abs(L2) + abs(L3)
    IK_leg_sum = abs(x) + abs(y) + abs(z)
    if IK_leg_sum >= LEG_SUM:
        print("Exceeded the leg limit")
        return None, None

    # Calculate joint angles using the calculate_joint_angles function
    angles = calculate_joint_angles(x, y, z)
    if not angles:
        print("Invalid target position")
        return
    
    # Extract joint angles
    theta_coxa, theta_femur, theta_tibia = angles
    print(f"Target: ({x}, {y}, {z}), Angles: (Coxa: {math.degrees(theta_coxa):.2f}°, Femur: {math.degrees(theta_femur):.2f}°, Tibia: {math.degrees(theta_tibia):.2f}°)")

    # Get the joint positions based on the angles
    points = get_leg_points(theta_coxa, theta_femur, theta_tibia)
    
    return points, theta_coxa, theta_femur, theta_tibia


def display_simulation():
    # Animate over time with the foot moving forward 1 unit at each step
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_zlim(-3, 3)
    
    # Add axis labels
    ax.set_xlabel('X Axis', labelpad=15)
    ax.set_ylabel('Y Axis', labelpad=15)
    ax.set_zlabel('Z Axis', labelpad=15)

    # Initialize starting position within the reachable range
    x_start, y_start, z_start = 2.5, 0.0, 0.5  # Starting position at the end of L1
    
    # Start the movement from (1, 0, 0)
    x = x_start
    y = y_start
    z = z_start
    
    # Set the target movement increment (1 unit forward)
    increment = -0.01
    
    # Loop to move the foot forward
    for i in range(100):  # For 100 steps (or adjust the number as needed        
        points, theta_coxa, theta_femur, theta_tibia = move_and_calculate_positions(x, y, z, increment, axis='x')
        
        # DEBUGGING STATEMENTS
        print(f"(x,y,z): {x:.2f}, {y:.2f}, {z:.2f}")
        print(f"(Coxa, Femur, Tibia): {math.degrees(theta_coxa):.2f}°, {math.degrees(theta_femur):.2f}°, {math.degrees(theta_tibia):.2f}°")
        print("")

        # Extract joint positions for plotting
        xs, ys, zs = zip(*points)
        
        # Update the plot
        ax.cla()  # Clear the axis to update the plot
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_zlim(-3, 3)
        
        # Plot the leg
        ax.plot(xs, ys, zs, marker='o', linewidth=3, markersize=5)
        
        # Plot the red dot at the tibia tip (last point p3)
        ax.scatter(xs[-1], ys[-1], zs[-1], color='red')  # Plot the red dot at tibia tip
        
        # Pause for animation effect
        plt.pause(0.05)

    plt.show()



# Main
display_simulation()