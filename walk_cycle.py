"""
motion/walk_cycle.py
--------------------
Generates foot-tip trajectories for a simple tripod-gait crawl.

Public API
----------
walk_cycle(dt=0.02, stride=2.0, lift=1.0, phase_a=(0,3,2), phase_b=(1,))
    • dt      – seconds per step that the generator yields
    • stride  – how far the body moves forward each full cycle (in)
    • lift    – foot lift height during swing (in)
    • phase_a – legs that support first  half-cycle
    • phase_b – legs that support second half-cycle
    Yields dict {leg_id: (x,y,z)} – XYZ targets in body frame.
"""

from __future__ import annotations
import math
from typing import Dict, Iterable, Tuple, List

# ----------------------------------------------------------------
# Geometry constants (import from config/constants if you have one)
COXA   = 2.0
FEMUR  = 2.5
TIBIA  = 5.0

HOME_Y = 5.99       # level, straight-out reach
HOME_Z =  2.66      # relaxed drop
HOME_X =  3.46

HOME_FOOT = {
    0: ( HOME_X,  HOME_Y,  HOME_Z),    # FR
    1: ( HOME_X,  HOME_Y,  HOME_Z),    # FL
    2: (-HOME_X,  HOME_Y,  HOME_Z),    # BR
    3: (-HOME_X,  HOME_Y,  HOME_Z),    # BL
}
# ----------------------------------------------------------------


def _s_curve(u: float) -> float:
    """0→1 S-curve (position),  v = (1-cosπu)/2"""
    return 0.5 * (1.0 - math.cos(math.pi * u))


def _swing_path(p0: Tuple[float,float,float],
                p1: Tuple[float,float,float],
                lift: float,
                u: float) -> Tuple[float,float,float]:
    """Foot path: S-curve in XY, half-cosine lift in Z."""
    x0,y0,z0 = p0
    x1,y1,z1 = p1
    s = _s_curve(u)
    x = x0 + (x1 - x0) * s
    y = y0 + (y1 - y0) * s
    z = z0 + (z1 - z0) * s - lift * math.sin(math.pi * s)
    return (x,y,z)


# ----------------------------------------------------------------
def walk_cycle(dt: float = 0.02,
               stride: float = 2.0,
               lift: float   = 1.0,
               phase_a: Iterable[int] = (0,3,2),
               phase_b: Iterable[int] = (1,)) -> Iterable[Dict[int, Tuple[float,float,float]]]:
    """
    Infinite generator of foot targets for a tripod crawl.
    """
    stance   : List[int] = list(phase_a)
    swing    : List[int] = list(phase_b)
    pose_cur : Dict[int, Tuple[float,float,float]] = HOME_FOOT.copy()

    swing_steps  = int(0.4 / dt)           # time spent in air  ≈0.4 s
    stance_steps = int(0.6 / dt)           # time chassis glides ≈0.6 s

    while True:
        # -------- 1) swing legs lift + move forward -------------
        p_start = {l: pose_cur[l] for l in swing}
        p_end   = {l: (x, y+stride, z) for l,(x,y,z) in p_start.items()}

        for i in range(swing_steps):
            u = i / swing_steps
            # stance legs stay planted
            yield {l: pose_cur[l] for l in stance} | \
                  {l: _swing_path(p_start[l], p_end[l], lift, u) for l in swing}
        # plant swing legs
        for l in swing:
            pose_cur[l] = p_end[l]

        # -------- 2) chassis glides forward ---------------------
        for i in range(stance_steps):
            dy = stride * (i / stance_steps)
            yield {l: (x, y-dy, z) for l,(x,y,z) in pose_cur.items()}
        # body finished glide; stance/swing swap
        stance, swing = swing, stance
