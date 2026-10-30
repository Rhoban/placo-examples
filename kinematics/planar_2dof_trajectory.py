import placo
import time
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, points_viz, robot_frame_viz, line_viz
from placo_utils.tf import tf

robot = placo.RobotWrapper("../models/planar-2dof/", placo.Flags.ignore_collisions)
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

# Adding loop closing task
closing_task = solver.add_relative_position_task("closing_effector_1", "closing_effector_2", np.zeros(3))
closing_task.configure("closing", "hard", 1.0)
closing_task.mask.set_axises("xy")

# Adding a task for the effector
T_world_effector = robot.get_T_world_frame("effector")
effector_task = solver.add_position_task("effector", T_world_effector[:3, 3])
effector_task.configure("effector", "soft", 1.0)
effector_task.mask.set_axises("yz")

# Building a square trajectory
trajectory = placo.CubicSpline3D()
x, y, z = T_world_effector[:3, 3].copy()
l = 0.05
trajectory.add_point(0, np.array([x, y+l, z-.075+l]), np.zeros(3))
trajectory.add_point(1, np.array([x, y+l, z-.075-l]), np.zeros(3))
trajectory.add_point(2, np.array([x, y-l, z-.075-l]), np.zeros(3))
trajectory.add_point(3, np.array([x, y-l, z-.075+l]), np.zeros(3))
trajectory.add_point(4, np.array([x, y+l, z-.075+l]), np.zeros(3))

viz = robot_viz(robot)
t = 0
dt = 0.01
last_targets = []
last_target_t = 0

@schedule(interval=dt)
def loop():
    global t, last_targets, last_target_t
    t += dt

    # Drawing a circle
    target = trajectory.pos(t % 4)
    effector_task.target_world = target

    # Solving the IK
    robot.update_kinematics()
    solver.solve(True)

    # Displaying the robot and the effector frame
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")

    # Drawing the last 50 targets (adding one point every 100ms)
    if t - last_target_t > 0.1:
        last_target_t = t
        last_targets.append(target)
        last_targets = last_targets[-20:]
        lines = []
        for i in range(len(last_targets) - 1):
            lines += [last_targets[i], last_targets[i+1]]
        line_viz("targets", np.array(lines), color=0xaaff00)



run_loop()