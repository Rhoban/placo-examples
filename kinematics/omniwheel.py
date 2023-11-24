import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz
from placo_utils.tf import tf

# Loading the robot
robot = placo.RobotWrapper("../models/omnidirectional", placo.Flags.ignore_collisions)
viz = robot_viz(robot)

# Creating the solver
solver = placo.KinematicsSolver(robot)

solver.enable_joint_limits(False)

# Control the base
base_task = solver.add_frame_task("base", np.eye(4))

# Omniwheel
wheel1_task = solver.add_wheel_task("wheel1", 0.04, True)
wheel2_task = solver.add_wheel_task("wheel2", 0.04, True)
wheel3_task = solver.add_wheel_task("wheel3", 0.04, True)

t = 0
dt = 0.01

@schedule(interval=dt)
def loop():
    global base_task, t, dt
    t += dt

    T_world_base = tf.translation_matrix([np.sin(t)*.25, np.cos(t)*.25, 0.05])
    base_task.T_world_frame = T_world_base

    robot.update_kinematics()
    solver.solve(True)
    viz.display(robot.state.q)
    robot_frame_viz(robot, "base")


run_loop()