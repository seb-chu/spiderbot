import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np

def plot_leg(joint_positions, ax):
    # Clear the plot before drawing new leg position
    ax.clear()

    # Extract x, y, and z coordinates of each joint from joint_positions
    x = [joint[0] for joint in joint_positions]
    y = [joint[1] for joint in joint_positions]
    z = [joint[2] for joint in joint_positions]

    # Plot the leg segments connecting the joints with markers at each joint
    ax.plot(x, y, z, "-o", linewidth=2, markersize=8)

    # Calculate the midpoints of each segment (Coxa, Femur, and Tibia)
    coxa_mid = [(joint_positions[0][i] + joint_positions[1][i])/2 for i in range(3)]
    femur_mid = [(joint_positions[1][i] + joint_positions[2][i])/2 for i in range(3)]
    tibia_mid = [(joint_positions[2][i] + joint_positions[3][i])/2 for i in range(3)]

    # Add labels for each segment at their respective midpoints
    ax.text(coxa_mid[0], coxa_mid[1], coxa_mid[2], 'Coxa', fontsize=12, color='blue')
    ax.text(femur_mid[0], femur_mid[1], femur_mid[2], 'Femur', fontsize=12, color='blue')
    ax.text(tibia_mid[0], tibia_mid[1], tibia_mid[2], 'Tibia', fontsize=12, color='blue')

    # Set labels for the x, y, and z axes
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # Set the title for the plot
    ax.set_title('Spider Leg Visualization')

    # Set plot limits to visualize all movement within a set space
    ax.set_xlim([-200, 200])  # X-axis limits
    ax.set_ylim([-200, 200])  # Y-axis limits
    ax.set_zlim([-200, 200])  # Z-axis limits


def animate_leg(leg, ax, fig, target, steps=50):
    # Step size for joint angles (linear interpolation)
    step_angles = [
        np.linspace(leg.theta1, leg.inverseKinematics(target=target)[0], steps),
        np.linspace(leg.theta2, leg.inverseKinematics(target=target)[1], steps),
        np.linspace(leg.theta3, leg.inverseKinematics(target=target)[2], steps)
    ]
    
    def update_frame(i):
        # Update joint angles for each step
        leg.setAngles([step_angles[0][i], step_angles[1][i], step_angles[2][i]])
        joint_positions = leg.forwardKinematics()
        plot_leg(joint_positions, ax)
        
    # Create animation
    ani = animation.FuncAnimation(fig, update_frame, frames=steps, interval=100, repeat=False)
    plt.show()
