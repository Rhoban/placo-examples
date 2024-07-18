import time
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz

"""
A differential gear system, demosntrating the use of gear task.
"""

robot = placo.RobotWrapper("../models/differential/", placo.Flags.ignore_collisions)
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

# Adding gears coupling constraints
gears = solver.add_gear_task()
gears.add_gear("alpha", "upper", 0.5)
gears.add_gear("alpha", "lower", 0.5)
gears.add_gear("beta", "upper", 1)
gears.add_gear("beta", "lower", -1)
gears.configure("gears", "hard")

# Adding a joints task
joints_task = solver.add_joints_task()
joints_task.set_joints({"lower": 0.0, "upper": 0.0})

viz = robot_viz(robot)

t: float = 0.0
dt: float = 0.01
lower: float = 0.0
upper: float = 0.0


@schedule(interval=dt)
def loop():
    global t, lower, upper
    t = (t + dt) % 8.0

    if t < 2.0:
        lower += dt
    elif t < 4.0:
        lower += dt
        upper -= dt
    elif t < 6.0:
        lower += dt
        upper += dt
    else:
        upper += dt

    joints_task.set_joint("lower", lower)
    joints_task.set_joint("upper", upper)

    solver.solve(True)
    robot.update_kinematics()

    viz.display(robot.state.q)


run_loop()
