import numpy as np
from ischedule import schedule, run_loop
import placo
from placo_utils.visualization import (
    robot_viz,
    contacts_viz,
)

"""
Tentative example of self collisions avoidance.
This feature is still quite experimental.
"""

robot = placo.RobotWrapper("../models/sigmaban/")

robot.update_kinematics()

solver = placo.DynamicsSolver(robot)

# Fixing the floating base
solver.mask_fbase(True)

# Joints task
joints_task = solver.add_joints_task()
joints_task.set_joints({joint: 0.0 for joint in robot.joint_names()})

# Self collisions constraint
constraint = solver.add_avoid_self_collisions_constraint()
constraint.self_collisions_margin = 0.025
constraint.self_collisions_trigger = 0.1
constraint.configure("cst", "hard")

viz = robot_viz(robot)

t = 0
solver.dt = 0.001
view_fps = 50  # FPS for viewer
steps_per_view = int((1 / view_fps) / solver.dt)


@schedule(interval=(1 / view_fps))
def loop():
    global t

    for _ in range(steps_per_view):
        t += solver.dt

        joints_task.set_joint("left_hip_roll", 0.5 * np.sin(t))

        robot.update_kinematics()
        solver.solve(True)

    # Viewing robot and contacts
    viz.display(robot.state.q)
    contacts_viz(solver, ratio=1e-2, radius=0.01)


run_loop()
