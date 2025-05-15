import pinocchio
import placo
import time
import numpy as np
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz
from placo_utils.tf import tf

"""
Orbita (3 axis rotation with loop closure constraint)
Controlled with task space (target orientation for the platform)
"""

robot = placo.RobotWrapper("../models/orbita/", placo.Flags.ignore_collisions)
robot.set_T_world_fbase(np.eye(4))
viz = robot_viz(robot)

solver = robot.make_solver()
solver.mask_fbase(True)
solver.enable_joint_limits(False)
solver.enable_velocity_limits(True)

cone_constraint = solver.add_cone_constraint("base", "effector", 0.4)
cone_constraint.configure("cone", "hard")

# Adding closure constraints
for closing in ["closing_ring2_br2", "closing_ring3_br2"]:
    task = solver.add_relative_position_task(
        f"{closing}_1", f"{closing}_2", np.array([0, 0, 0])
    )
    task.configure(closing, "hard")
    task.mask.set_axises("xy")

# Creating a task to control orientation
orientation_task = solver.add_orientation_task("effector", np.eye(3))

dt = 0.01
solver.dt = dt
t = 0

while True:
    # Updating the task's target
    pos = robot.get_T_world_frame("effector")[0:3, 3]
    T = tf.translation_matrix(pos) @ (
        tf.rotation_matrix(t, [0, 0, 1])
        @ tf.rotation_matrix(np.sin(t * 5) * 1.2, [0, 1, 0])
        @ tf.rotation_matrix(t * 4, [0, 0, 1])
    )
    orientation_task.R_world_frame = T[:3, :3]
    frame_viz("T", T, 0.25)

    # Solving kinematics
    solver.solve(True)
    robot.update_kinematics()
    solver.dump_status()

    # Displaying
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")
    robot_frame_viz(robot, "base")

    t += dt
    time.sleep(dt)
