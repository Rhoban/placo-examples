import pinocchio
import numpy as np
from ischedule import schedule, run_loop
import placo
from placo_utils.visualization import (
    robot_viz,
    contacts_viz,
)

"""
The robot base is attached to the world, with zero torque in the joints
(only some damping is added for stability).

An external wrench is applied to the robot's right foot, getting it moving.
"""

robot = placo.RobotWrapper("../models/sigmaban/")

robot.update_kinematics()

solver = placo.DynamicsSolver(robot)

# Fixing the floating base
solver.mask_fbase(True)

# Setting the torque of all the joints to zero with some damping
torque_task = solver.add_torque_task()
for joint in robot.joint_names():
    if "head" not in joint:
        torque_task.set_torque(joint, 0.0, 0.0, 5e-1)

# Joints task
joints_task = solver.add_joints_task()
joints_task.set_joints({"head_pitch": 0.0, "head_yaw": 0.0})

# External wrench contact
external_wrench = solver.add_external_wrench_contact("right_foot", "world")
external_wrench.w_ext = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])

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

        external_wrench.w_ext = np.array([np.sin(t) * 10.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        robot.update_kinematics()
        solver.solve(True)

    # Viewing robot and contacts
    viz.display(robot.state.q)
    contacts_viz(solver, ratio=1e-2, radius=0.01)


run_loop()
