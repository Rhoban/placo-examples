import pinocchio
import placo
import numpy as np
import argparse
from ischedule import schedule, run_loop
from placo_utils.tf import tf
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz

"""
UR5 following a given trajectory, basic example for the dynamics solver.
"""

args_parser = argparse.ArgumentParser()
args_parser.add_argument(
    "--no-velocity", help="Disable velocity tracking", action="store_true"
)
args = args_parser.parse_args()

# Loading the robot
robot = placo.RobotWrapper("../models/ur5", placo.Flags.ignore_collisions)

# Creating the solver
solver = placo.DynamicsSolver(robot)
solver.mask_fbase(True)

effector_task = solver.add_frame_task("ee_link", np.eye(4))
effector_task.configure("ee_link", "soft", 1.0, 1.0)

# Enable velocity limits
solver.enable_torque_limits(True)
solver.enable_velocity_limits(True)
solver.enable_joint_limits(True)

viz = robot_viz(robot)


def get_trajectory(t: float):
    # Pulsation
    w = 1.0
    # Amplitude [m]
    amplitude = 0.35

    # Target effector pose (4x4 matrix)
    T_world_effector = tf.translation_matrix([0.4, amplitude * np.sin(t * w), 0.3])

    # Target effector position velocity
    dtarget_world = np.array([0.0, amplitude * w * np.cos(t * w), 0.0])

    return T_world_effector, dtarget_world


t = 0
solver.dt = 0.001
view_fps = 50  # FPS for viewer
steps_per_view = int((1 / view_fps) / solver.dt)


@schedule(interval=(1 / view_fps))
def loop():
    global t

    # Updating the solver (10 times)
    for _ in range(steps_per_view):
        t += solver.dt

        T_world_effector, dtarget_world = get_trajectory(t)
        effector_task.T_world_frame = T_world_effector
        if not args.no_velocity:
            effector_task.position().dtarget_world = dtarget_world

        # Solving the IK
        solver.solve(True)
        robot.update_kinematics()

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "ee_link")
    frame_viz("target", effector_task.T_world_frame)


run_loop()
