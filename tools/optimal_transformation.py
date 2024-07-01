import placo
import numpy as np
import time
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, point_viz

"""
This is a demonstration of the use of placo.optimal_transformation, to find the best
transformation between two sets of points.

In this example, expected support for the four legs of a quadruped robot are used to
estimate the position of the floating base of the robot in the world.

The supports in the world are moved to see the result of different fitting.
"""

robot = placo.RobotWrapper("../models/megabot/", placo.Flags.ignore_collisions)
robot.update_kinematics()

viz = robot_viz(robot)
t = 0
dt = 0.02

@schedule(interval=dt)
def loop():
    global t
    viz.display(robot.state.q)

    # Updating joint limits and velocity limits
    radius = 1.2 + 0.3 * np.sin(t)
    rotation = np.sin(t)*0.25
    points_world = [
        [
            np.cos(angle + rotation) * radius + np.cos(1.2 * t) * 0.2,
            np.sin(angle + rotation) * radius + np.sin(1.7 * t) * 0.2,
            0.0,
        ]
        for angle in [-3 * np.pi / 4, 3 * np.pi / 4, np.pi / 4, -np.pi / 4]
    ]

    points_base = [robot.get_T_a_b("base", f"leg_{leg}")[:3, 3] for leg in range(1, 5)]

    T_world_base = placo.optimal_transformation(
        np.array(points_world), np.array(points_base)
    )
    robot.set_T_world_frame("base", T_world_base)
    robot.update_kinematics()

    # Target points
    for k, target in enumerate(points_world):
        point_viz(f"target_{k}", target, radius=0.1)

    # Leg frames
    for leg in range(1, 5):
        robot_frame_viz(robot, f"leg_{leg}")

    t += dt

run_loop()