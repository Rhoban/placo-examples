import time
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz

"""
A differential gear system, demosntrating the use of gear task.
"""

robot = placo.RobotWrapper("../models/differential/", placo.Flags.ignore_collisions)
solver = placo.DynamicsSolver(robot)
solver.mask_fbase(True)

# Adding gears coupling constraints
gears = solver.add_gear_task()
gears.add_gear("alpha", "upper", 0.5)
gears.add_gear("alpha", "lower", 0.5)
gears.add_gear("beta", "upper", 1)
gears.add_gear("beta", "lower", -1)
gears.configure("gears", "hard")
gears_contact = solver.add_task_contact(gears)

# Enforcing passive DoFs to have zero torque
torque_task = solver.add_torque_task()
torque_task.set_torque("alpha", 0.0, 0.0, 1e-4)
torque_task.set_torque("beta", 0.0, 0.0, 1e-4)
torque_task.configure("passive", "hard")

# Adding a joints task
joints_task = solver.add_joints_task()
joints_task.set_joints({"lower": 0.0, "upper": 0.0})

viz = robot_viz(robot)

t = -5
solver.dt = 0.005
view_fps = 25  # FPS for viewer
steps_per_view = int((1 / view_fps) / solver.dt)
lower: float = 0.0
upper: float = 0.0

solver.enable_torque_limits(True)


@schedule(interval=(1 / view_fps))
def loop():
    global t, lower, upper

    for _ in range(steps_per_view):
        t += solver.dt

        if t < 2.0:
            lower += solver.dt
        elif t < 4.0:
            lower += solver.dt
            upper -= solver.dt
        elif t < 6.0:
            lower += solver.dt
            upper += solver.dt
        elif t < 8.0:
            upper -= solver.dt
        else:
            # After 8s, forcing the motors torque to be 0
            torque_task.set_torque("lower", 0.0)
            torque_task.set_torque("upper", 0.0)

        joints_task.set_joint("lower", lower)
        joints_task.set_joint("upper", upper)

        solver.solve(True)
        robot.update_kinematics()

    viz.display(robot.state.q)


run_loop()
