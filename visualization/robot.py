from placo_utils.visualization import robot_viz, robot_frame_viz
from ischedule import schedule, run_loop
import numpy as np
import placo

"""
Visualizes a 6-axis robot
"""

robot = placo.RobotWrapper("../models/6axis/", placo.Flags.ignore_collisions)
viz = robot_viz(robot)

t = 0
dt = 0.01

@schedule(interval=dt)
def loop():
    global t
    t += dt

    # Moving some DofS
    robot.set_joint('r1', np.sin(t))
    robot.set_joint('r2', np.sin(t/3))

    # Updating kinematics
    robot.update_kinematics()

    # Showing effector frame
    robot_frame_viz(robot, "effector")

    # Updating the viewer
    viz.display(robot.state.q)

run_loop()