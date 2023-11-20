from placo_utils.visualization import arrow_viz, get_viewer
from ischedule import schedule, run_loop
import numpy as np

"""
Visualizes a custom arrow
"""

t = 0
dt = 0.01


@schedule(interval=dt)
def loop():
    global t
    t += dt

    x = np.cos(t) * 0.25
    y = np.sin(t) * 0.25
    z = 0.3

    arrow_viz(
        "arrow",
        np.array([0.0, 0.0, 0.0]),
        np.array([x, y, z]),
        color=0xff1100,
        radius=0.01,
    )


run_loop()
