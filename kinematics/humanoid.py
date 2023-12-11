import time
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, point_viz, robot_frame_viz

"""
Sigmaban humanoid is moving its legs while looking at a moving ball.
"""

# Loading the robot
robot = placo.HumanoidRobot("../models/sigmaban/")
robot.set_T_world_frame("left_foot", np.eye(4))
robot.update_kinematics()

robot.set_joint("left_knee", 0.1)
robot.set_joint("right_knee", 0.1)

solver = robot.make_solver()

# Retrieving initial position of the feet, com and trunk orientation
T_world_left = robot.get_T_world_frame("left_foot")
T_world_right = robot.get_T_world_frame("right_foot")

# Creating the viewer
viz = robot_viz(robot)

# Trunk
T_world_trunk = robot.get_T_world_frame("trunk")
T_world_trunk[2, 3] = 0.35
trunk_task = solver.add_frame_task("trunk", T_world_trunk)
trunk_task.configure("trunk_task", "soft", 1.0, 1.0)

# Keep left and right foot on the floor
left_foot_task = solver.add_frame_task("left_foot", T_world_left)
left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)
right_foot_task.T_world_frame = T_world_right

# Look at ball
look_at_ball = solver.add_axisalign_task(
    "camera", np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0])
)
look_at_ball.configure("look_ball", "soft", 1.0)

# Creating a very basic lateral swing and foot rise trajectory
left_foot_z_traj = placo.CubicSpline()
left_foot_z_traj.add_point(0.0, 0.0, 0.0)
left_foot_z_traj.add_point(1.0, 0.05, 0.0)
left_foot_z_traj.add_point(2.0, 0.0, 0.0)
left_foot_z_traj.add_point(4.0, 0.0, 0.0)

right_foot_z_traj = placo.CubicSpline()
right_foot_z_traj.add_point(0.0, 0.0, 0.0)
right_foot_z_traj.add_point(2.0, 0.0, 0.0)
right_foot_z_traj.add_point(3.0, 0.05, 0.0)
right_foot_z_traj.add_point(4.0, 0.0, 0.0)

trunk_y_traj = placo.CubicSpline()
initial_trunk_y = T_world_trunk[1, 3]
trunk_y_traj.add_point(-1.0, initial_trunk_y + 0.05, 0.0)
trunk_y_traj.add_point(1.0, initial_trunk_y - 0.05, 0.0)
trunk_y_traj.add_point(3.0, initial_trunk_y + 0.05, 0.0)
trunk_y_traj.add_point(5.0, initial_trunk_y - 0.05, 0.0)

# Setting custom target values for elbows
posture_regularization_task = solver.add_joints_task()
posture_regularization_task.set_joints(
    {dof: 0.0 for dof in robot.actuated_joint_names()}
)
posture_regularization_task.configure("reg", "soft", 1e-5)

solver.enable_joint_limits(True)
solver.enable_velocity_limits(True)

t = 0
dt = 0.01
last = 0
solver.dt = dt
start_t = time.time()
robot.update_kinematics()


@schedule(interval=dt)
def loop():
    global t

    # Updating the target
    t_mod = t % 4.0
    target = left_foot_task.position().target_world
    target[2] = left_foot_z_traj.pos(t_mod)
    left_foot_task.position().target_world = target

    target = right_foot_task.position().target_world
    target[2] = right_foot_z_traj.pos(t_mod)
    right_foot_task.position().target_world = target

    target = trunk_task.position().target_world
    target[1] = trunk_y_traj.pos(t_mod)
    trunk_task.position().target_world = target

    # Looking at ball
    ball = np.array([0.5 + np.cos(t) * 0.25, np.sin(t) * 0.7, 0.0])
    camera_pos = robot.get_T_world_frame("camera")[:3, 3]
    look_at_ball.R_world_frame = placo.rotation_from_axis("z", ball - camera_pos)
    look_at_ball.targetAxis_world = ball - camera_pos

    robot.update_kinematics()
    solver.solve(True)

    viz.display(robot.state.q)
    robot_frame_viz(robot, "trunk")
    robot_frame_viz(robot, "camera")
    point_viz("com", robot.com_world(), radius=0.025, color=0xAAAAAA)
    point_viz("ball", ball, radius=0.05, color=0xdddddd)

    t += dt


run_loop()
