import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kinematics import kinematics
from math import radians, sin, cos

# Initialize kinematics object
k = kinematics()

# Set up the figure and 3D axis
fig = plt.figure(figsize=(10, 8))
ax = plt.axes(projection="3d")
ax.set_xlim(-0.3, 0.3)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(-0.3, 0.3)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# Plot the base
k.plot_base(ax)

# Define leg ID
leg_id = 0  # Left front leg

# Generate a trajectory for the leg
def generate_trajectory(num_points=50, radius=0.05, center_x=-0.05, center_y=0.15, center_z=-0.15):
    """Generate a circular trajectory for the leg to follow"""
    t = np.linspace(0, 2*np.pi, num_points)
    x = center_x + radius * np.cos(t)
    y = center_y + radius * np.sin(t)
    z = center_z + 0.02 * np.sin(2*t)  # Add some up and down movement
    return np.array([x, y, z]).T

# Generate the trajectory points
trajectory = generate_trajectory()

# Initialize the line that will represent the leg
leg_line, = ax.plot([], [], [], 'b', linewidth=2)

# Function to update the leg position for animation
def update(frame):
    # Get the target position for this frame
    target_xyz = trajectory[frame]
    
    # Calculate the leg joints using inverse kinematics
    ik_result = k.leg_IK(target_xyz, legID=leg_id)
    j1, j2, j3, j4 = ik_result[3:]
    
    # Get the leg origin offset
    origin = k.leg_origins[leg_id].A1  # Convert matrix to array
    
    # Create points array for the line
    # Note: j1 is already [0,0,0] relative to the leg origin
    points = np.vstack([
        origin,             # Origin point
        origin,             # J1 (same as origin)
        origin + j2,        # J2
        origin + j3,        # J3
        origin + j4         # J4 (end effector)
    ])
    
    # Update the leg line data
    x_data = points[:, 0]
    y_data = points[:, 1]
    z_data = points[:, 2]
    
    leg_line.set_data(x_data, y_data)
    leg_line.set_3d_properties(z_data)
    
    # Clear previous end effector markers
    for collection in ax.collections:
        if hasattr(collection, 'end_effector_marker'):
            collection.remove()
    
    # Add a small marker at the end effector position
    scatter = ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], color='red', s=50)
    scatter.end_effector_marker = True
    
    return leg_line,

# Create the animation
animation = FuncAnimation(
    fig, update, frames=len(trajectory), 
    interval=100, blit=True, repeat=True
)

# Add the trajectory line to the plot
ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'r--', alpha=0.5)

# Set a nice view angle
ax.view_init(elev=30, azim=45)

plt.title("Leg Movement Simulation")
plt.tight_layout()
plt.show()