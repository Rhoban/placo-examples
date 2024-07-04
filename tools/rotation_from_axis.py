import placo
import numpy as np
from placo_utils.visualization import frame_viz, arrow_viz
from ischedule import schedule, run_loop


"""
Example for rotation_from_axis function.
Generates a rotation matrix where a given axis points toward a given direction.
"""

t = 0
dt = 0.02


@schedule(interval=dt)
def loop():
    global t

    # Switching axis from x, y, z, x, y, z, ...
    axis = "xyz"[int(t / (2 * np.pi)) % 3]

    v = np.array([np.cos(t), np.sin(t), 1.0]) / 10.0

    T_world_target = np.eye(4)
    T_world_target[:3, :3] = placo.rotation_from_axis(axis, v)

    arrow_viz("target_direction", np.array([0.0, 0.0, 0.0]), v, color=0xDD55FF)

    frame_viz("target", T_world_target)
    t += dt


run_loop()
