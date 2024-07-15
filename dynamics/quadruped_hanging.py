import numpy as np
import placo
from ischedule import schedule, run_loop
from placo_utils.tf import tf
from placo_utils.visualization import (
    robot_viz,
    contacts_viz,
)

"""
In this example, a quadruped is attached with fixed points to the ground.
All the dofs are not actuated (torque is set to zero), only one is controlled with a sinusoidal trajectory.

In results in an "hanging" robot.
"""

robot = placo.RobotWrapper("../models/quadruped/")

# Putting the robot in a standing position
robot.update_kinematics()
robot.set_T_world_frame("trunk", tf.translation_matrix([0.0, 0.0, 0.13]))
for k in range(1, 5):
    robot.set_joint(f"leg{k}_b", 0.5)
    robot.set_joint(f"leg{k}_c", 0.5)
robot.update_kinematics()

solver = placo.DynamicsSolver(robot)

for k in range(1, 5):
    leg_world = robot.get_T_world_frame(f"leg{k}")[:3, 3]
    leg_task = solver.add_position_task(f"leg{k}", leg_world)
    leg_task.configure(f"leg{k}_pos", "soft", 1.0)
    leg_task.kp = 1e5
    contact = solver.add_point_contact(leg_task)

# Enabling all the limits
solver.enable_torque_limits(True)
solver.enable_velocity_limits(True)

controlled_dof = "leg1_b"

# Creating a joints task
torque_task = solver.add_torque_task()
for joint in robot.joint_names():
    if joint != controlled_dof:
        torque_task.set_torque(joint, 0.0, 0.0, 1e-3)

# Actuating one DoF
joints_task = solver.add_joints_task()
joints_task.set_joint(controlled_dof, 0.0)

viz = robot_viz(robot)

t = 0
solver.dt = 0.001
view_fps = 50  # FPS for viewer
steps_per_view = int((1 / view_fps) / solver.dt)


@schedule(interval=(1 / view_fps))
def loop():
    global t

    for _ in range(steps_per_view):
        t += solver.dt

        joints_task.set_joint(controlled_dof, np.sin(t) * 1.0)

        solver.solve(True)
        robot.update_kinematics()

    viz.display(robot.state.q)
    contacts_viz(solver)


run_loop()
