import time, math
from kinematics import kinematics
from plot import walk_plot
# --- geometry -----------------------
STEP_LENGTH = 3.0      # inches, total fore-aft swing
LIFT_HEIGHT = 1.0      # inches, peak foot lift (+ up; IK expects –z)
CYCLE_TIME  = 1.0      # seconds for one full step
DT          = 0.02     # control period  (50 Hz)

# Link Lengths (inches)
COXA_LINK = 2.0
FEMUR_LINK = 2.5
TIBIA_LINK = 5

# Servo channels (change if yours are plugged in elsewhere)
COXA_CHANNEL = 0
FEMUR_CHANNEL = 1
TIBIA_CHANNEL = 2

# straight-leg “home” pose
HOME_X = 0.0
HOME_Y = COXA_LINK + FEMUR_LINK + TIBIA_LINK
HOME_Z = 0.0

# ------------------------------------
def walk_cycle():
    """infinite generator of (x,y,z) foot targets"""
    t = 0.0
    while True:
        phase = (t % CYCLE_TIME) / CYCLE_TIME  # 0…1
        if phase < 0.5:                        # STANCE
            p  = phase / 0.5                   # 0…1
            dx = -STEP_LENGTH * (p - 0.5)      # +½L → –½L
            dz = 0.0
        else:                                  # SWING
            p  = (phase - 0.5) / 0.5           # 0…1
            dx =  STEP_LENGTH * (p - 0.5)      # –½L → +½L
            dz =  LIFT_HEIGHT * math.sin(math.pi * p)

        # IK uses –z for downward
        yield (HOME_X + dx,
               HOME_Y,
               HOME_Z - dz,
               t)                              # include time for debug

        t += DT

# ------------------------------------
for x, y, z, t in walk_cycle():
    # --- send to inverse kinematics ---
    # kinematics(x, y, z)

    # --- DEBUG: show absolute + relative offsets -------------
    rel_x = x - HOME_X
    rel_y = y - HOME_Y
    rel_z = z - HOME_Z
    print(f"t={t:4.2f}  pos=({x:+5.2f}, {y:+5.2f}, {z:+5.2f})"
          f"  Δ=({rel_x:+5.2f}, {rel_y:+5.2f}, {rel_z:+5.2f})")

    # walk_plot_one_leg()
    time.sleep(DT)
