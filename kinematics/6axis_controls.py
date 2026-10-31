import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, task_controls

"""
6axis robot whose effector target can be moved from the viewer: drag the arrows to move it, and the rings to
rotate it.
- Velocity and joint limits are enforced

(Depending on the robot configuration, the target may not be reachable)
"""

# Loading the robot
robot = placo.RobotWrapper("../models/6axis", placo.Flags.ignore_collisions)

# Starting from a bent configuration
for joint, value in {"r2": 0.4, "r3": 0.8, "r5": 0.6}.items():
    robot.set_joint(joint, value)
robot.update_kinematics()

# Creating the solver
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)
solver.enable_velocity_limits(True)

# The target is the current effector pose, it is then moved from the viewer
effector_task = solver.add_frame_task("effector", robot.get_T_world_frame("effector"))
effector_task.configure("effector", "soft", 1.0, 1.0)

viz = robot_viz(robot)
task_controls(effector_task, "effector")

dt = 0.01
solver.dt = dt


@schedule(interval=dt)
def loop():
    # Solving the IK
    solver.solve(True)
    robot.update_kinematics()

    # Displaying the robot and effector
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")


run_loop()
