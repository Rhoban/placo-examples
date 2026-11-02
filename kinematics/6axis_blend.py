import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz, add_slider

"""
6axis robot with two tasks: a joints task (a given configuration, shown as a ghost of the robot) and a frame task
(a given effector pose, shown as a frame). The "alpha" slider blends their weights: the joints task has weight
1 - alpha and the frame task alpha, so the robot transitions from one to the other.
"""

# Loading the robot
robot = placo.RobotWrapper("../models/6axis/robot.urdf", placo.Flags.ignore_collisions)

# Creating the solver
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)
solver.enable_velocity_limits(True)

# Joints task: a configuration of the robot
joints = {"r1": 0.8, "r2": 0.3, "r3": 0.2, "r4": 0.0, "r5": 0.6, "r6": 0.0}
joints_task = solver.add_joints_task()
joints_task.set_joints(joints)

# Frame task: the effector pose of another configuration (so that it is reachable)
for joint, value in {"r1": -0.8, "r2": 0.4, "r3": -0.3, "r4": 0.5, "r5": -0.6, "r6": 0.3}.items():
    robot.set_joint(joint, value)
robot.update_kinematics()
effector_task = solver.add_frame_task("effector", robot.get_T_world_frame("effector"))

# Starting from the joints task configuration
for joint, value in joints.items():
    robot.set_joint(joint, value)
robot.update_kinematics()

# The robot, and a ghost of the robot in the joints task configuration
viz = robot_viz(robot)
ghost_viz = robot_viz(robot, "joints_target", color=0x3399FF, opacity=0.35)
q_joints = robot.state.q.copy()

# Slider blending the two tasks
alpha_slider = add_slider("alpha", 0.0, 1.0, 0.0)

dt = 0.01
solver.dt = dt


@schedule(interval=dt)
def loop():
    # Weights of the tasks
    alpha = alpha_slider.value
    joints_task.configure("joints", "soft", 1.0 - alpha)
    effector_task.configure("effector", "soft", alpha, alpha)

    # Solving the IK
    solver.solve(True)
    robot.update_kinematics()

    # Displaying the robot, the joints task target (ghost) and the frame task target
    viz.display(robot.state.q)
    ghost_viz.display(q_joints)
    frame_viz("effector_target", effector_task.T_world_frame)
    robot_frame_viz(robot, "effector")


run_loop()
