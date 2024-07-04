import placo
import time
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import *
from placo_utils.tf import tf

"""
Usage example for placo.interpolate_frames, which interpolates between two frames.

This will result in an animation with the interpolated frame moving between two example frames.
"""


T_world_frameA = tf.translation_matrix([0.1, -0.2, 0.3]) @ tf.rotation_matrix(
    np.pi / 4, [0, 0, 1]
)
T_world_frameB = tf.translation_matrix([0.2, 0.4, 0.4]) @ tf.rotation_matrix(
    1.3 * np.pi, [0, 1.0, 0]
)
t = 0
dt = 0.01


@schedule(interval=dt)
def loop():
    global t
    t += dt
    t = t % 4

    if t < 2:
        alpha = t / 2
    else:
        alpha = 1 - (t - 2) / 2

    T_world_interpolate = placo.interpolate_frames(
        T_world_frameA, T_world_frameB, alpha
    )

    frame_viz("frameA", T_world_frameA)
    frame_viz("frameB", T_world_frameB)
    frame_viz("interpolated", T_world_interpolate, opacity=0.5)


run_loop()
