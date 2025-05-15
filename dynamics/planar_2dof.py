import pinocchio
import numpy as np
from ischedule import schedule, run_loop
import placo
from placo_utils.visualization import robot_viz, robot_frame_viz

"""
In this example, a planar 2dof parallel robot is controlled with the dynamics solver.

The loop closure is handled by a relative position task with contact at the effector.
Passive degrees of freedom are controlled by a zero-torque task.
The motor1 joint is controlled by a sinusoidal function.
"""

robot = placo.RobotWrapper("../models/planar-2dof/", placo.Flags.ignore_collisions)

viz = robot_viz(robot)

solver = placo.DynamicsSolver(robot)
solver.mask_fbase(True)
solver.enable_torque_limits(True)

# Adding a relative position task with contact at the effector
loop_closure = solver.add_relative_position_task(
    "closing_effector_1", "closing_effector_2", np.array([0.0, 0.0, 0.0])
)
loop_closure.mask.set_axises("xy")
loop_closure.configure("closure", "hard")
loop_closure_contact = solver.add_task_contact(loop_closure)

# Imposing zero torque on passive degrees of freedom
torque_task = solver.add_torque_task()
torque_task.set_torque("passive1", 0.0)
torque_task.set_torque("passive2", 0.0)
torque_task.configure("torque", "hard")

# Creating a joints task for control
joints_task = solver.add_joints_task()
joints_task.set_joints({"motor1": 0.0, "motor2": 0.0})

t = 0
solver.dt = 0.001
view_fps = 50  # FPS for viewer
steps_per_view = int((1 / view_fps) / solver.dt)


@schedule(interval=(1 / view_fps))
def loop():
    global t

    for _ in range(steps_per_view):
        t += solver.dt

        joints_task.set_joint("motor1", np.sin(t) * 0.5)

        solver.solve(True)
        robot.update_kinematics()

    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")


run_loop()
