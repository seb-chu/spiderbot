import math
import numpy as np

# link lengths (inches)
COXA_LINK  = 2.0
FEMUR_LINK = 2.5
TIBIA_LINK = 5.0

def forward_kinematics(coxa_deg, femur_deg, tibia_deg):
    """Return (x,y,z) foot coordinates for given joint angles."""
    # ---- convert to radians -----------------------------------
    yaw   = math.radians(coxa_deg)
    pitch = math.radians(femur_deg)
    knee  = math.radians(tibia_deg)

    # ---- base → hip (coxa) ------------------------------------
    hip = np.array([
        COXA_LINK * math.sin(yaw),   # x
        COXA_LINK * math.cos(yaw),   # y
        0.0                          # z
    ])

    # ---- hip → knee (femur) -----------------------------------
    # unit vector of coxa plane
    in_plane = np.array([math.sin(yaw), math.cos(yaw), 0.0])
    in_plane /= np.linalg.norm(in_plane)

    dir_femur = ( math.cos(pitch) * in_plane        # forward component
                + np.array([0, 0, -math.sin(pitch)]) )  # vertical
    knee_pt = hip + dir_femur * FEMUR_LINK

    # ---- knee → foot (tibia) ----------------------------------
    # tibia pivots in same plane as femur
    # knee flexion 0° = straight; negative = fold down
    tibial_pitch = pitch + knee           # total downward bend
    dir_tibia = ( math.cos(tibial_pitch) * in_plane
                + np.array([0,0,-math.sin(tibial_pitch)]) )
    foot = knee_pt + dir_tibia * TIBIA_LINK

    return tuple(foot)

# ---------------- example -------------------------------------
# coxa 20° to the left, femur lifted 15°, tibia folded -30°
px, py, pz = forward_kinematics(30, 35, -90)
print(f"foot at (x,y,z) = {px:.2f}, {py:.2f}, {pz:.2f}")
