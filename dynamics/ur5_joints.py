import pinocchio
import placo
import numpy as np
import argparse
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, contacts_viz

"""
UR5 arm is following a sinusoidal joint-space trajectory with a fixed contact on the base.
"""

args_parser = argparse.ArgumentParser()
args_parser.add_argument(
    "--no-velocity", help="Disable velocity tracking", action="store_true"
)
args = args_parser.parse_args()

# Loading the robot
robot = placo.RobotWrapper("../models/ur5", placo.Flags.ignore_collisions)
robot.set_T_world_frame("base", np.eye(4))
robot.update_kinematics()

# Creating the solver
solver = placo.DynamicsSolver(robot)

base_task = solver.add_frame_task("base", np.eye(4))
base_task.configure("base", "hard", 1.0, 1.0)
base_contact = solver.add_fixed_contact(base_task)

joints = solver.add_joints_task()
joints.set_joints({joint: 0.0 for joint in robot.joint_names()})
joints.set_joint("elbow_joint", -np.pi / 4)

# Enable velocity limits
solver.enable_torque_limits(True)
solver.enable_velocity_limits(True)
solver.enable_joint_limits(True)

viz = robot_viz(robot)

t = 0
solver.dt = 0.001
view_fps = 50  # FPS for viewer
steps_per_view = int((1 / view_fps) / solver.dt)


@schedule(interval=(1 / view_fps))
def loop():
    global t, goal_positions, positions

    # Updating the solver (10 times)
    for _ in range(steps_per_view):
        t += solver.dt

        # Setting target position, velocity and acceleration
        joints.set_joint("shoulder_pan_joint", np.sin(t), np.cos(t), -np.sin(t))

        # Solving the IK
        solver.solve(True)
        robot.update_kinematics()

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "ee_link")
    contacts_viz(solver, 2e-3, 0.02)


run_loop()
