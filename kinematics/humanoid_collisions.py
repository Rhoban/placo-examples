import time
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz
from placo_utils.tf import tf

"""
Sigmaban humanoid is moving its legs while looking at a moving ball.
"""

# Loading the robot
robot = placo.HumanoidRobot("../models/sigmaban/")

# Placing the left foot in world origin
robot.set_joint("left_knee", 0.1)
robot.set_joint("right_knee", 0.1)
robot.update_kinematics()
robot.set_T_world_frame("left_foot", np.eye(4))
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)

# Retrieving initial position of the feet, com and trunk orientation
T_world_left = robot.get_T_world_frame("left_foot")
T_world_right = robot.get_T_world_frame("right_foot")

# Creating the viewer
viz = robot_viz(robot)

# Trunk
T_world_trunk = robot.get_T_world_frame("trunk")
T_world_trunk[2, 3] = 0.35
trunk_task = solver.add_frame_task("trunk", T_world_trunk)
trunk_task.configure("trunk_task", "soft", 1e3, 1e3)

# Keep left and right foot on the floor
left_foot_task = solver.add_frame_task("left_foot", T_world_left)
left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot", "soft", 1e3, 1e3)

# Regularization task
posture_regularization_task = solver.add_joints_task()
posture_regularization_task.set_joints(
    {dof: 0.0 for dof in robot.joint_names()}
)
posture_regularization_task.configure("reg", "soft", 1e-5)

# Initializing robot position before enabling constraints
for _ in range(32):
    robot.update_kinematics()
    solver.solve(True)

# Enabling joint and velocity limits
solver.enable_joint_limits(True)
solver.enable_velocity_limits(True)

# Enabling self collisions avoidance
avoid_self_collisions = solver.add_avoid_self_collisions_constraint()
avoid_self_collisions.configure("avoid_self_collisions", "hard")

# The constraint starts existing when contacts are 3cm away, and keeps a 1cm margin
avoid_self_collisions.self_collisions_margin = 0.01 # [m] 
avoid_self_collisions.self_collisions_trigger = 0.03 # [m]

t = 0
dt = 0.01
last = 0
solver.dt = dt
start_t = time.time()
robot.update_kinematics()


@schedule(interval=dt)
def loop():
    global t

    # Updating left foot target
    left_foot_task.T_world_frame = tf.translation_matrix(
        [np.sin(t * 2.5) * 0.05, np.sin(t * 3) * 0.1, 0.04]
    ) @ tf.rotation_matrix(np.sin(t) * 0.25, [1, 0, 0])

    robot.update_kinematics()
    solver.solve(True)

    viz.display(robot.state.q)
    robot_frame_viz(robot, "left_foot")
    frame_viz("left_foot_target", left_foot_task.T_world_frame, opacity=0.25)

    t += dt


run_loop()
