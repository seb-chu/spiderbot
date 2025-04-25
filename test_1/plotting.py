import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_leg(joint_positions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x = [joint[0] for joint in joint_positions]
    y = [joint[1] for joint in joint_positions]
    z = [joint[2] for joint in joint_positions]

    ax.plot(x, y, z, "-o", linewidth=2, markersize=8)

    coxa_mid = [(joint_positions[0][i] + joint_positions[1][i])/2 for i in range(3)]
    femur_mid = [(joint_positions[1][i] + joint_positions[2][i])/2 for i in range(3)]
    tibia_mid = [(joint_positions[2][i] + joint_positions[3][i])/2 for i in range(3)]

    ax.text(coxa_mid[0], coxa_mid[1], coxa_mid[2], 'Coxa', fontsize=12, color='blue')
    ax.text(femur_mid[0], femur_mid[1], femur_mid[2], 'Femur', fontsize=12, color='blue')
    ax.text(tibia_mid[0], tibia_mid[1], tibia_mid[2], 'Tibia', fontsize=12, color='blue')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    ax.set_title('Spider Leg Visualization')

    plt.show()
