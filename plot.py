import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ------- geometry (same constants you use) --------------------
COXA_LINK  = 2.0
FEMUR_LINK = 2.5
TIBIA_LINK = 5.0

def forward_leg(coxa_deg, femur_deg, foot_xyz):
    """
    Very simple forward sketch:
      • base at (0,0,0)
      • coxa rotates in XY by yaw = coxa_deg
      • femur pitches in the vertical plane aligned with coxa
      • tibia simply connects knee→foot point we already know from IK
    Returns a list of points [base, hip, knee, foot]
    """
    base = np.array([0.0, 0.0, 0.0])

    # hip (end of coxa link)
    yaw   = np.radians(coxa_deg)
    hip = np.array([
        COXA_LINK * np.sin(yaw),
        COXA_LINK * np.cos(yaw),
        0.0
    ])

    # femur direction in the same plane
    pitch = np.radians(femur_deg)     # 0° femur = horizontal
    in_plane = np.array([np.sin(yaw), np.cos(yaw), 0.0])  # unit in XY
    in_plane /= np.linalg.norm(in_plane)
    dir_femur = (np.cos(pitch) * in_plane) + np.array([0,0,-np.sin(pitch)])
    knee = hip + dir_femur * FEMUR_LINK

    foot = np.array(foot_xyz)
    return np.vstack([base, hip, knee, foot])

# -------------------- demo ------------------------------------
def kinematics_plot(pos_x, pos_y, pos_z):
    # ---- run your IK to get joint angles ----------------
    h = np.hypot(pos_x, pos_y)
    l = np.hypot(pos_z, h - COXA_LINK)
    coxa_deg  = np.degrees(np.arctan2(pos_x, pos_y))
    hip_elev  = np.arctan2(-pos_z, h - COXA_LINK)
    b         = np.arccos((FEMUR_LINK**2 + l**2 - TIBIA_LINK**2)/(2*FEMUR_LINK*l))
    femur_deg = np.degrees(hip_elev + b)
    # tibia not needed for drawing because we know foot

    pts = forward_leg(coxa_deg, femur_deg, (pos_x, pos_y, pos_z))

    # ---- plot -------------------------------------------
    fig = plt.figure(figsize=(5,5))
    ax  = fig.add_subplot(111, projection='3d')
    ax.plot(pts[:,0], pts[:,1], pts[:,2], '-o', lw=2)

    # label
    ax.scatter(0,0,0, c='k', s=30)
    ax.text(0,0,0,"Coxa")

    # nicer view
    ax.set_xlabel('X (left/right)')
    ax.set_ylabel('Y (forward)')
    ax.set_zlabel('Z (up +)')
    ax.set_title("Leg sketch from IK")
    ax.set_box_aspect([1,1,1])
    lim = COXA_LINK+FEMUR_LINK+TIBIA_LINK+1
    ax.set_xlim(-lim, lim); ax.set_ylim(0, lim); ax.set_zlim(-lim*0.5, lim*0.5)
    plt.tight_layout()
    plt.show()

# ---- call with two poses ---------------------------
kinematics_plot(0, FEMUR_LINK+TIBIA_LINK+COXA_LINK, 0)   # straight ahead
kinematics_plot(0, 9, -1.87)  # yaw + down
