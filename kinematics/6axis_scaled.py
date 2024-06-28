import placo
import numpy as np
import pinocchio as pin
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz

"""
6axis robot drawing a circle in scaled mode

(Depending on the robot configuration, the target may not be reachable)
"""

# Loading the robot
robot = placo.RobotWrapper("../models/6axis", placo.Flags.ignore_collisions)
robot.set_joint("r5", 1.0)  # getting out of default singularity

# Creating the solver
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

effector_task = solver.add_frame_task("effector", np.eye(4))
effector_task.configure("effector", "scaled", 1.0, 1.0)

# Enable velocity limits
solver.enable_velocity_limits(True)

viz = robot_viz(robot)

t = 0
last_target = 0
dt = 0.01
solver.dt = dt


@schedule(interval=dt)
def loop():
    global t, last_target

    # The target trajectory is a very fast circle, that the robot is not able to follow
    effector_task.position().target_world = np.array(
        [1.5, np.cos(t * 10) * 0.5, 0.75 + np.sin(t * 10) * 0.5]
    )

    # Solving the IK
    robot.update_kinematics()
    solver.solve(True)

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")
    frame_viz("target", effector_task.T_world_frame)

    # After solving, the scale gives us the amount of task actually achieved, we can then adjust the trajectory reading
    t += dt * solver.scale
    print(f"Scale={solver.scale*100}%")


run_loop()
