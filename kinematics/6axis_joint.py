import pinocchio
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz

"""
6axis robot, using a joint task to move r1
"""

# Loading the robot
robot = placo.RobotWrapper("../models/6axis", placo.Flags.ignore_collisions)

# Creating the solver
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

joints_task = solver.add_joints_task()
joints_task.set_joints({f"r{i}": 0.0 for i in range(1, 7)})

viz = robot_viz(robot)

t = 0
dt = 0.01
solver.dt = dt


@schedule(interval=dt)
def loop():
    global t
    t += dt

    # Updating the target
    joints_task.set_joints({"r1": np.sin(t)})

    # Solving the IK
    solver.solve(True)
    robot.update_kinematics()

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")


run_loop()
