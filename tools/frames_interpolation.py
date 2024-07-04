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


T_world_frame1 = tf.translation_matrix([0.1, -0.2, 0.3]) @ tf.rotation_matrix(
    np.pi / 4, [0, 0, 1]
)
T_world_frame2 = tf.translation_matrix([0.2, 0.4, 0.4]) @ tf.rotation_matrix(
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
        T_world_frame1, T_world_frame2, alpha
    )

    frame_viz("frame1", T_world_frame1)
    frame_viz("frame2", T_world_frame2)
    frame_viz("interpolated", T_world_interpolate, opacity=0.5)


run_loop()
