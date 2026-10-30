import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, line_viz, task_controls
from placo_utils.tf import tf

"""
Quadruped robot whose free leg (leg3) and body targets can be moved from the viewer:
- Standing on three legs (leg1, leg2 and leg4) - hard priority
- leg3 target (position task) - high priority, drag the arrows to move it
- Body target (frame task) - low priority, drag the arrows and rings to move and rotate it
- Avoiding tilting (CoM is constrained in the support) - hard priority
- Velocity are constrained to 1 rad/s
"""

robot = placo.RobotWrapper("../models/quadruped", placo.Flags.ignore_collisions)

for leg in ["leg1", "leg2", "leg3", "leg4"]:
    robot.set_joint_limits(leg + "_a", -np.pi / 2, np.pi / 2)
    robot.set_joint_limits(leg + "_c", 0.0, np.pi)

solver = placo.KinematicsSolver(robot)

leg1 = solver.add_position_task("leg1", np.array([-0.15, 0.0, 0.0]))
leg2 = solver.add_position_task("leg2", np.array([0.02, -0.15, 0.0]))
leg3 = solver.add_position_task("leg3", np.array([0.15, 0.0, 0.0]))
leg4 = solver.add_position_task("leg4", np.array([0.02, 0.15, 0.0]))

body_task = solver.add_frame_task("body", tf.translation_matrix([0.0, 0.0, 0.05]))

# Using some steps to initialize the robot
for _ in range(32):
    robot.update_kinematics()
    solver.solve(True)

# Support legs should not move (hard constraint)
leg1.configure("leg1", "hard")
leg2.configure("leg2", "hard")
leg4.configure("leg4", "hard")
support_tasks = [leg1, leg4, leg2]
for k in range(3):
    line_from = support_tasks[k].target_world
    line_to = support_tasks[(k + 1) % 3].target_world
    line_viz(f"support_{k}", np.array([line_from, line_to]), color=0xFFAA00)

# The body should remain (soft constraint)
body_task.configure("body", "soft", 1.0, 1.0)

# The leg3 should reach its target (soft constraint, higher priority than body), starting above the ground
leg3.configure("leg3", "soft", 1e3)
leg3.target_world = np.array([0.2, 0.0, 0.1])

# Adding a polygon constraint to avoid robot tilting
polygon = np.array([[-0.15, 0.0], [0.02, 0.15], [0.02, -0.15]])
com = solver.add_com_polygon_constraint(polygon, 0.015)
com.configure("com_constraint", "hard")

# Limiting velocities to 1 rad/s
robot.set_velocity_limits(1.0)
solver.enable_velocity_limits(True)

viz = robot_viz(robot)

# The leg3 and body targets can be moved from the viewer
task_controls(leg3, "leg3", scale=0.1)
task_controls(body_task, "body", scale=0.1)

dt = 0.01
solver.dt = dt


@schedule(interval=dt)
def loop():
    solver.solve(True)
    robot.update_kinematics()

    viz.display(robot.state.q)


run_loop()
