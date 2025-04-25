import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kinematics import kinematics
from math import radians, sin, cos, pi

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

# Generate a walking gait trajectory with a downward parabola
def generate_walking_gait(num_points=100, 
                          stride_length=0.1,  # Length of one complete stride
                          stride_height=0.04,  # Max height of swing phase
                          ground_height=-0.15,  # Height of the ground
                          leg_forward_offset=0.05,  # Forward offset of leg from origin
                          leg_side_offset=0.15):   # Side offset of leg from origin
    """
    Generate a walking gait trajectory with a downward parabola during swing phase.
    The gait consists of:
    1. Stance phase (on ground, moving backward) - 60% of cycle
    2. Swing phase (in air, moving forward) - 40% of cycle
    """
    # Points in each phase
    stance_points = int(0.6 * num_points)
    swing_points = num_points - stance_points
    
    # Initialize arrays
    x = np.zeros(num_points)
    y = np.zeros(num_points)
    z = np.zeros(num_points)
    
    # Stance phase (leg on ground, moving backward)
    stance_x = np.linspace(leg_forward_offset - stride_length/2, 
                           leg_forward_offset + stride_length/2, 
                           stance_points)
    x[:stance_points] = stance_x
    y[:stance_points] = leg_side_offset
    z[:stance_points] = ground_height
    
    # Swing phase (leg in air, moving forward with parabola)
    swing_x = np.linspace(leg_forward_offset + stride_length/2, 
                          leg_forward_offset - stride_length/2, 
                          swing_points)
    x[stance_points:] = swing_x
    y[stance_points:] = leg_side_offset
    
    # Parabolic trajectory for swing phase
    # Normalize the x position to calculate parabola height
    norm_x = np.linspace(0, 1, swing_points)
    # Parabola equation: h = 4 * max_height * x * (1 - x)
    # This creates a parabola with max height at x=0.5
    swing_z = ground_height + 4 * stride_height * norm_x * (1 - norm_x)
    z[stance_points:] = swing_z
    
    return np.array([x, y, z]).T

# Generate the trajectory points
trajectory = generate_walking_gait()

# Initialize the line that will represent the leg
leg_line, = ax.plot([], [], [], 'b', linewidth=2)

# Create container for the end effector trail
trail_x, trail_y, trail_z = [], [], []
trail_line, = ax.plot([], [], [], 'r-', alpha=0.3)

# Function to update the leg position for animation
def update(frame):
    # Get the target position for this frame
    target_xyz = trajectory[frame]
    
    # Calculate the leg joints using inverse kinematics
    ik_result = k.leg_IK(target_xyz, legID=leg_id)
    coxa_angle, femur_angle, tibia_angle, coxa_joint, femur_joint, tibia_joint, end_effector = ik_result
    
    # Get the leg origin offset
    origin = k.leg_origins[leg_id].A1  # Convert matrix to array
    
    # Create points array for the leg line
    points = np.vstack([
        origin,               # Origin point
        origin,               # Coxa joint (same as origin)
        origin + femur_joint, # Femur joint
        origin + tibia_joint, # Tibia joint
        origin + end_effector # End effector
    ])
    
    # Update the leg line data
    leg_line.set_data(points[:, 0], points[:, 1])
    leg_line.set_3d_properties(points[:, 2])
    
    # Update trail for end effector
    if frame % 2 == 0:  # Only add every other point to avoid cluttering
        trail_x.append(points[-1, 0])
        trail_y.append(points[-1, 1])
        trail_z.append(points[-1, 2])
    
    trail_line.set_data(trail_x, trail_y)
    trail_line.set_3d_properties(trail_z)
    
    # Clear previous end effector markers
    for collection in ax.collections:
        if hasattr(collection, 'end_effector_marker'):
            collection.remove()
    
    # Add a small marker at the end effector position
    scatter = ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], color='red', s=50)
    scatter.end_effector_marker = True
    
    # Add text to indicate swing vs stance phase
    if frame > 0.6 * len(trajectory):
        phase_text = ax.text2D(0.05, 0.95, "Swing Phase", transform=ax.transAxes, color='green')
    else:
        phase_text = ax.text2D(0.05, 0.95, "Stance Phase", transform=ax.transAxes, color='blue')
    
    # Return all artists that need to be redrawn
    return leg_line, trail_line, scatter, phase_text

# Create the animation
animation = FuncAnimation(
    fig, update, frames=len(trajectory), 
    interval=50, blit=True, repeat=True
)

# Set a nice view angle
ax.view_init(elev=30, azim=120)

plt.title("Leg Walking Gait Cycle")
plt.tight_layout()

# Add ground plane visualization
ground_x = np.linspace(-0.3, 0.3, 10)
ground_y = np.linspace(-0.3, 0.3, 10)
ground_xx, ground_yy = np.meshgrid(ground_x, ground_y)
ground_z = np.ones_like(ground_xx) * trajectory[0, 2]  # Ground height
ax.plot_surface(ground_xx, ground_yy, ground_z, alpha=0.2, color='gray')

plt.show()

# Uncomment to save the animation
# animation.save('leg_walking_gait.gif', writer='pillow', fps=20)   