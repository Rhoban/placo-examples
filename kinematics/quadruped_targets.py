import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, line_viz, point_viz, frame_viz, robot_frame_viz
from placo_utils.tf import tf

"""
Quadruped robot:
- Standing on three legs (leg1, leg2 and leg4) - hard priority
- Trying to reach targets with leg3 (randomized every 3s) - high priority
- Trying to keep its body - low priority
- Avoiding tilting (CoM is constrained in the support) - hard priority
- Velocity are constrained to 1 rad/s
"""

robot = placo.RobotWrapper("../models/quadruped", placo.Flags.ignore_collisions)

for leg in ["leg1", "leg2", "leg3", "leg4"]:
    robot.set_joint_limits(leg + "_a", -np.pi/2, np.pi/2)
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

# The leg3 should reach its targets (soft constraint, higher priority than body)
leg3.configure("leg3", "soft", 1e3)

# Adding a polygon constraint to avoid robot tilting
polygon = np.array([
    [-0.15, 0.],
    [0.02, 0.15],
    [0.02, -0.15]
])
com = solver.add_com_polygon_constraint(polygon, 0.015)
com.configure("com_constraint", "hard")

# Limiting velocities to 1 rad/s
robot.set_velocity_limits(1.)

solver.enable_velocity_limits(True)
viz = robot_viz(robot)

t = 0.0
dt = 0.01
solver.dt = dt


def sample_target():
    return np.random.uniform(np.array([0.1, -0.1, 0.0]), np.array([0.3, 0.1, 0.2]))


target = sample_target()
last_sample_t = 0.0


@schedule(interval=dt)
def loop():
    global t, target, last_sample_t
    t += dt

    # Updating target every 3 seconds
    if last_sample_t + 3.0 < t:
        last_sample_t = t
        target = sample_target()

    # Showing target
    point_viz("target", target, color=0x00FF00)
    leg3.target_world = target

    # Showing the center of mass (on the ground)
    com_world = robot.com_world()
    com_world[2] = 0.0
    point_viz("com", com_world, color=0xFF0000)

    # Showing body frame and its target
    robot_frame_viz(robot, "body")
    frame_viz("body_target", body_task.T_world_frame, opacity=.25)

    robot.update_kinematics()
    solver.solve(True)

    viz.display(robot.state.q)


run_loop()
