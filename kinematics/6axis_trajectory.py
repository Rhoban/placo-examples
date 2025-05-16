import pinocchio
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz, points_viz
from placo_utils.tf import tf

"""
6axis robot following an infinity sign (∞) trajectory
"""

# Loading the robot
robot = placo.RobotWrapper("../models/6axis", placo.Flags.ignore_collisions)

# Creating the solver
solver = placo.KinematicsSolver(robot)

# The floating base is fixed (robot is not moving)
solver.mask_fbase(True)

# Creating a task for the effector
effector_task = solver.add_frame_task("effector", np.eye(4))
effector_task.configure("effector", "soft", 10.0, 1.0)

# Enable joints velocity limits
solver.enable_velocity_limits(True)

viz = robot_viz(robot)
t = 0
dt = 0.01
solver.dt = dt
last_targets = []
last_target_t = 0

@schedule(interval=dt)
def loop():
    global t, last_targets, last_target_t
    t += dt

    # Updating the effector task (drawing an infinite sign ∞)
    target = [1.5, np.cos(t) * 0.5, 0.75 + np.sin(2 * t) * 0.25]
    effector_task.T_world_frame = tf.translation_matrix(target)

    # Solving the IK
    solver.solve(True)
    robot.update_kinematics()

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")
    frame_viz("target", effector_task.T_world_frame)

    # Drawing the last 50 targets (adding one point every 100ms)
    if t - last_target_t > 0.1:
        last_target_t = t
        last_targets.append(target)
        last_targets = last_targets[-50:]
        points_viz("targets", last_targets, color=0xaaff00)


run_loop()
