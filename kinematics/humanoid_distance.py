import pinocchio
import time
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz

"""
An humanoid robot is moving its right leg, which is constrained to remain at less than 30cm from the trunk.
"""

# Loading the robot
robot = placo.HumanoidRobot("../models/sigmaban/")

# Placing the left foot in world origin
robot.set_T_world_frame("trunk", np.eye(4))
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)

trunk_task = solver.add_frame_task("trunk", np.eye(4))
trunk_task.configure("trunk", "soft", 1e3)

T_world_right = robot.get_T_world_frame("right_foot")
right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)

# Regularization task
posture_regularization_task = solver.add_joints_task()
posture_regularization_task.set_joints(
    {dof: 0.0 for dof in robot.joint_names()}
)
posture_regularization_task.configure("reg", "soft", 1e-6)

solver.enable_joint_limits(True)

# Reaching initial configuration
for k in range(128):
    robot.add_q_noise(1e-3)
    robot.update_kinematics()
    solver.solve(True)
    robot.update_kinematics()

distance_constraint = solver.add_distance_constraint("trunk", "right_foot", 0.3)
distance_constraint.configure("distance", "hard")

viz = robot_viz(robot)
t = 0
dt = 0.01
last = 0
solver.dt = dt
start_t = time.time()
robot.update_kinematics()


@schedule(interval=dt)
def loop():
    global t

    target = np.array([np.sin(t)*0.2, -0.05, -0.28])
    right_foot_task.position().target_world = target

    robot.add_q_noise(1e-3)
    robot.update_kinematics()
    solver.solve(True)
    solver.dump_status()
    robot.update_kinematics()

    viz.display(robot.state.q)
    robot_frame_viz(robot, "trunk")
    robot_frame_viz(robot, "right_foot")
    frame_viz("target", right_foot_task.T_world_frame, 0.5)

    t += dt


run_loop()
