import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz

"""
6axis robot moving to target configurations, shown as a transparent "ghost" of the robot
"""

# Loading the robot
robot = placo.RobotWrapper("../models/6axis/robot.urdf", placo.Flags.ignore_collisions)

# Creating the solver
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)
solver.enable_velocity_limits(True)

joints_task = solver.add_joints_task()

# Target configurations (joints r1 to r6), reached one after the other
targets = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.2, 0.4, -0.3, 0.5, 0.8, 0.0],
    [-0.9, -0.3, 0.6, -0.6, -0.5, 1.0],
]

# The robot, and a "ghost" of the robot (blue, transparent) showing the target configuration
viz = robot_viz(robot)
target_viz = robot_viz(robot, "target", color=0x3399FF, opacity=0.35)

t = 0
dt = 0.01
solver.dt = dt


@schedule(interval=dt)
def loop():
    global t
    t += dt

    # Changing target every 3 seconds
    target = targets[int(t / 3) % len(targets)]
    joints_task.set_joints({f"r{i + 1}": value for i, value in enumerate(target)})

    # Solving the IK
    solver.solve(True)
    robot.update_kinematics()

    # Displaying the robot, and the ghost in the target configuration
    viz.display(robot.state.q)
    q_target = robot.state.q.copy()
    for i, value in enumerate(target):
        q_target[robot.get_joint_offset(f"r{i + 1}")] = value
    target_viz.display(q_target)


run_loop()
