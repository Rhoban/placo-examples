import placo
import time
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz, points_viz
from placo_utils.tf import tf

"""
SPM inverse kinematics, using Polytope workspace limits.
Limits were computed using workspace_estimation.py, and stored in workspace.pkl.

An orientation task is used, asking 
"""

robot = placo.RobotWrapper("../models/spm/", placo.Flags.ignore_collisions)

solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

for closing in ["closing_m1_p2", "closing_m3_p2"]:
    closing_task = solver.add_relative_position_task(
        f"{closing}_1", f"{closing}_2", np.zeros(3)
    )
    closing_task.configure("closing", "hard", 1.0)
    closing_task.mask.set_axises("xy")

from polytope import Polytope

polytope = Polytope.load("workspace.pkl")

A = np.zeros((len(polytope.A), len(robot.state.q)))
A[:, robot.get_joint_offset("m1")] = polytope.A[:, 0]
A[:, robot.get_joint_offset("m2")] = polytope.A[:, 1]
A[:, robot.get_joint_offset("m3")] = polytope.A[:, 2]
cst = solver.add_joint_space_half_spaces_constraint(A, polytope.b)
cst.configure("workspace", "hard")

orientation = solver.add_orientation_task("effector", np.eye(3))

solver.add_regularization_task(1e-3)

viz = robot_viz(robot)
t = 0.0
dt = 0.01

initial_q = robot.state.q.copy()


@schedule(interval=dt)
def loop():
    global t
    t += dt

    orientation.R_world_frame = tf.rotation_matrix(
        np.sin(t * 3) * 1.0, [0.0, 1.0, 0.0]
    )[:3, :3]

    robot.update_kinematics()
    solver.solve(True)

    robot_frame_viz(robot, "effector")
    viz.display(robot.state.q)

    T_world_target = robot.get_T_world_frame("effector")
    T_world_target[:3, :3] = orientation.R_world_frame
    frame_viz("target", T_world_target, opacity=0.25)


run_loop()
