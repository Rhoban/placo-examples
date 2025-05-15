import pinocchio
import placo
import time
import tqdm
import numpy as np
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz, points_viz

"""
The SPM workspace is estimated. For a set of directions in joint-space, the solver is invoked with
increasing distances until a the target is either unreachable or collision occurs.
"""

# Whether to debug (Meshcat viewer)
debug = False
# How many directions are used
n_directions = 256

robot = placo.RobotWrapper("../models/spm/")
robot.load_collision_pairs("../models/spm/collision_pairs.json")

solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

for closing in ["closing_m1_p2", "closing_m3_p2"]:
    closing_task = solver.add_relative_position_task(
        f"{closing}_1", f"{closing}_2", np.zeros(3)
    )
    closing_task.configure("closing", "hard", 1.0)
    closing_task.mask.set_axises("xy")

joints = solver.add_joints_task()

solver.add_regularization_task(1e-3)

viz = robot_viz(robot)
t = 0.0

initial_q = robot.state.q.copy()


def find_highest_distance(direction, max_distance=2):
    robot.reset()
    robot.update_kinematics()

    for distance in np.linspace(0, max_distance, 100):
        joints.set_joints(
            {
                "m1": direction[0] * distance,
                "m2": direction[1] * distance,
                "m3": direction[2] * distance,
            }
        )

        for _ in range(4):
            solver.solve(True)
            robot.update_kinematics()

        if debug:
            robot_frame_viz(robot, "effector")
            viz.display(robot.state.q)

        collisions = robot.self_collisions(False)
        points_viz(
            "collisions",
            [c.get_contact(0) for c in collisions],
            radius=0.003,
            color=0xFF0000,
        )

        if joints.error_norm() > 1e-5:
            if debug:
                input(
                    f"Target unreachable for direction {direction}, press [ENTER] to continue"
                )
            return distance

        if collisions:
            if debug:
                input(
                    f"Collision detected for direction {direction}, press [ENTER] to continue"
                )
            return distance

        if debug:
            time.sleep(0.01)

    return max_distance


import tqdm

points = []
directions = placo.directions_3d(n_directions)

print(f"Sampling workspace, using {n_directions} directions")
for direction in tqdm.tqdm(directions):
    dist = find_highest_distance(direction)
    points.append(direction * dist)

import pickle

pickle.dump(points, open("points.pkl", "wb"))
