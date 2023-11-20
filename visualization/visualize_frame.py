from placo_utils.visualization import frame_viz
from placo_utils.tf import tf
from ischedule import schedule, run_loop
import numpy as np

"""
Visualizes custom frames
"""

t = 0
dt = 0.01

@schedule(interval=dt)
def loop():
    global t
    t += dt

    # A is rotating around the origin z-axis, at a distance of 0.2m
    T_world_a = tf.rotation_matrix(t, [0, 0, 1]) @ tf.translation_matrix([0.2, 0, 0])
    frame_viz("a", T_world_a)

    # B is rotating around the origin x-axis
    T_world_b = tf.rotation_matrix(t, [1, 0, 0])
    frame_viz("b", T_world_b)

    # C is steady, translated at 0.25m on the z-axis
    T_world_c = tf.translation_matrix([0, 0, 0.25])
    frame_viz("c", T_world_c)

run_loop()