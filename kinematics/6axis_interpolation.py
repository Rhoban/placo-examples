import pinocchio
import placo
import numpy as np
import pinocchio as pin
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz

"""
Example using interpolate_frames and a CubicSpline to produce a smooth trajectory
"""

# Loading the robot
robot = placo.RobotWrapper("../models/6axis", placo.Flags.ignore_collisions)

# Creating the solver
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

effector_task = solver.add_frame_task("effector", np.eye(4))
effector_task.configure("effector", "soft", 1.0, 1.0)

# Enable velocity limits
solver.enable_velocity_limits(True)

viz = robot_viz(robot)


def update_target():
    # Choose a random target
    r = pin.exp6(np.array([0.0, 0.0, 0.0, *np.random.uniform([-1.0] * 3, [1.0] * 3)]))
    t = pin.exp6(
        np.array([*np.random.uniform([1.0, -1.0, 0.5], [1.5, 1.0, 1.2]), 0.0, 0.0, 0.0])
    )

    return np.array(t * r)


t = 0
dt = 0.01
solver.dt = dt
targets_duration = 3.0
spline = placo.CubicSpline()
spline.add_point(0.0, 0.0, 0.0)
spline.add_point(targets_duration, 1.0, 0.0)
start_pose = robot.get_T_world_frame("effector")
next_pose = update_target()


@schedule(interval=dt)
def loop():
    global t, start_pose, next_pose
    t += dt

    if t > 3:
        t = 0
        start_pose = robot.get_T_world_frame("effector")
        next_pose = update_target()

    effector_task.T_world_frame = placo.interpolate_frames(
        start_pose, next_pose, spline.pos(t)
    )

    # Solving the IK
    solver.solve(True)
    robot.update_kinematics()

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")
    frame_viz("target", next_pose)


run_loop()
