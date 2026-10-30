import placo
import time
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, points_viz, robot_frame_viz, line_viz
from placo_utils.tf import tf

robot = placo.RobotWrapper("../models/planar-2dof/", placo.Flags.ignore_collisions)
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)
solver.enable_velocity_limits(True)

# Adding loop closing task
closing_task = solver.add_relative_position_task("closing_effector_1", "closing_effector_2", np.zeros(3))
closing_task.configure("closing", "hard", 1.0)
closing_task.mask.set_axises("xy")

# Adding a gear between the joints
gear_task = solver.add_gear_task()
gear_task.configure("gear", "hard")
gear_task.set_gear("motor2", "motor1", -1.0)

# Adding a task for the joints
joints_task = solver.add_joints_task()
joints_task.set_joints({"motor1": 0,})
joints_task.configure("joints", "soft", 1.0)

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

    # Moving motor 1
    joints_task.set_joints({"motor1": np.sin(t)*.3})

    # Solving the IK
    solver.solve(True)
    robot.update_kinematics()
    solver.dump_status()

    # Displaying the robot and the effector frame
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")



run_loop()
