import pinocchio
import time
import numpy as np
import pinocchio
import placo
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz, points_viz
from placo_utils.tf import tf

"""
Spherical parallel manipulator, a 3 DoF
- Two revolute loop closures are handled as hard constraints
- Robot is fixed, so floating base is fixed
- A single task controls the effector orientation
"""

robot = placo.RobotWrapper("../models/spm/")
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

for closing in ["closing_m1_p2", "closing_m3_p2"]:
    closing_task = solver.add_relative_position_task(
        f"{closing}_1", f"{closing}_2", np.zeros(3)
    )
    closing_task.configure("closing", "hard", 1.0)
    closing_task.mask.set_axises("xy")

orientation = solver.add_orientation_task("effector", np.eye(3))

solver.add_regularization_task(1e-3)

viz = robot_viz(robot)
t = 0.0
dt = 0.01
solver.dt = dt


@schedule(interval=dt)
def loop():
    global t
    t += dt

    orientation.R_world_frame = tf.rotation_matrix(
        np.sin(t * 3) * 0.3, [0.0, 1.0, 0.0]
    )[:3, :3]

    robot.update_kinematics()
    solver.solve(True)

    robot_frame_viz(robot, "effector")
    viz.display(robot.state.q)


run_loop()
