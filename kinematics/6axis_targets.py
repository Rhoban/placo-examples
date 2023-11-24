import placo
import numpy as np
import pinocchio as pin
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz

# Loading the robot
robot = placo.RobotWrapper("../models/6axis", placo.Flags.ignore_collisions)

# Creating the solver
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

effector_task = solver.add_frame_task("effector", np.eye(4))
effector_task.configure("effector", "scaled", 10.0, 1.0)

# Enable velocity limits
solver.enable_velocity_limits(True)

viz = robot_viz(robot)


def update_target():
    global effector_task

    # Choose a random target
    r = pin.exp6(np.array([0.0, 0.0, 0.0, *np.random.uniform([-1.0] * 3, [1.0] * 3)]))
    t = pin.exp6(
        np.array([*np.random.uniform([1.0, -1.0, 0.5], [1.5, 1.0, 1.2]), 0.0, 0.0, 0.0])
    )

    effector_task.T_world_frame = np.array(t * r)


t = 0
last_target = 0
dt = 0.01
solver.dt = dt
update_target()


@schedule(interval=dt)
def loop():
    global t, last_target
    t += dt

    if t > last_target + 3:
        # Updating the target every 3 seconds
        last_target = t
        update_target()

    # Solving the IK
    robot.update_kinematics()
    solver.solve(True)

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")
    frame_viz("target", effector_task.T_world_frame)


run_loop()
