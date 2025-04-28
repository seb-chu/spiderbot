# walk_cycle.py

import numpy as np

def interpolate_points(start, end, steps=10):
    """
    Generate intermediate points between start and end.

    start, end: (x, y, z) coordinates
    steps: number of points between start and end (including start and end)
    """
    return [
        tuple(start + (end - start) * (i / float(steps)))
        for i in range(steps + 1)
    ]

def generate_smooth_walk(points, steps=10):
    """
    Generate a list of smooth points based on the key points for the walk cycle.
    """
    smooth_points = []
    for i in range(len(points) - 1):
        smooth_points.extend(interpolate_points(np.array(points[i]), np.array(points[i+1]), steps))
    return smooth_points
