import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.tf import tf
from placo_utils.visualization import (
    robot_viz,
    robot_frame_viz,
    contacts_viz,
)

"""
In this example, an UR5 arm is following a sinusoidal trajectory with a fixed contact on the base.
"""

# Loading the robot
robot = placo.RobotWrapper("../models/ur5", placo.Flags.ignore_collisions)
robot.update_kinematics()
robot.set_T_world_frame("base", np.eye(4))

# Creating the solver
solver = placo.DynamicsSolver(robot)

base_task = solver.add_frame_task("base", np.eye(4))
base_task.configure("base", "hard", 1.0, 1.0)
base_contact = solver.add_fixed_contact(base_task)

effector_task = solver.add_frame_task("ee_link", np.eye(4))
effector_task.configure("ee_link", "soft", 1.0, 1.0)

# Enable velocity limits
solver.enable_torque_limits(True)
solver.enable_velocity_limits(True)
solver.enable_joint_limits(True)

viz = robot_viz(robot)

t = 0
dt = 0.01
refine = 10
solver.dt = dt / refine


@schedule(interval=dt)
def loop():
    global t

    # Updating the solver (10 times)
    for _ in range(refine):
        t += dt / refine

        effector_task.T_world_frame = tf.translation_matrix(
            [0.4, 0.25 * np.sin(t * 3), 0.4]
        )

        # Solving the IK
        solver.solve(True)
        robot.update_kinematics()

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "ee_link")
    contacts_viz(solver, 2e-3, 0.02)


run_loop()
