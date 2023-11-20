from placo_utils.visualization import point_viz
from ischedule import schedule, run_loop
import numpy as np

"""
Visualizes a point moving on a circle of radius 0.25m
"""

t = 0
dt = 0.01

@schedule(interval=dt)
def loop():
    global t
    t += dt

    point_viz(
        "point", 0.25 * np.array([np.cos(t), np.sin(t), 0]), radius=0.03, color=0x00FF00
    )

run_loop()