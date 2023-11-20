from placo_utils.visualization import line_viz
from placo_utils.tf import tf
from ischedule import schedule, run_loop
import numpy as np

"""
Visualizes a custom polygon
"""

t = 0
dt = 0.01


@schedule(interval=dt)
def loop():
    global t
    t += dt

    lines = (
        np.array(
            [
                [0.0, 1.0, 0],
                [1.0, 0.0, 0],
                [1.0, 0.0, 0],
                [0.0, -1.0, 0],
                [0.0, -1.0, 0],
                [0.0, 1.0, 0],
            ]
        )
        * 0.25
    )

    R = tf.rotation_matrix(t, [0, 0, 1])[:3, :3]
    lines = (R @ lines.T).T

    line_viz("polygon", lines, color=0xFF11AA)


run_loop()
