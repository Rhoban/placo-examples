from placo_utils.visualization import point_viz, points_viz
from ischedule import schedule, run_loop
import numpy as np

"""
Visualizes a point moving on a circle of radius 0.25m, and a set of points
"""

t = 0
dt = 0.01


@schedule(interval=dt)
def loop():
    global t
    t += dt

    # Building a single point
    point_viz(
        "point", 0.25 * np.array([np.cos(t), np.sin(t), 0]), radius=0.03, color=0x00FF00
    )

    # Building a set of points
    xs = np.linspace(-0.5, 0.5, 8)
    zs = 0.2 + 0.1 * np.cos(xs * 6 + t * 2)
    points = [[x, 0, z] for x, z in zip(xs, zs)]

    points_viz("points", points, radius=0.03, color=0x0000FF)


run_loop()
