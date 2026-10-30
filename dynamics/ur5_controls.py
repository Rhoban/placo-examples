import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, task_controls

"""
UR5 controlled with the dynamics solver, whose effector target can be moved from the viewer: drag the arrows to move
it, and the rings to rotate it.
- Torque, velocity and joint limits are enforced
"""

# Loading the robot
robot = placo.RobotWrapper("../models/ur5", placo.Flags.ignore_collisions)

# Starting from a bent configuration
for joint, value in {"shoulder_lift_joint": -1.2, "elbow_joint": 1.5, "wrist_1_joint": -1.8, "wrist_2_joint": -1.57}.items():
    robot.set_joint(joint, value)
robot.update_kinematics()

# Creating the solver
solver = placo.DynamicsSolver(robot)
solver.mask_fbase(True)
solver.enable_torque_limits(True)
solver.enable_velocity_limits(True)
solver.enable_joint_limits(True)

# The target is the current effector pose, it is then moved from the viewer
effector_task = solver.add_frame_task("ee_link", robot.get_T_world_frame("ee_link"))
effector_task.configure("ee_link", "soft", 1.0, 1.0)

viz = robot_viz(robot)
task_controls(effector_task, "ee_link")

solver.dt = 0.001
view_fps = 50  # FPS for viewer
steps_per_view = int((1 / view_fps) / solver.dt)


@schedule(interval=(1 / view_fps))
def loop():
    # Updating the solver
    for _ in range(steps_per_view):
        solver.solve(True)
        robot.update_kinematics()

    # Displaying the robot and effector
    viz.display(robot.state.q)
    robot_frame_viz(robot, "ee_link")


run_loop()
