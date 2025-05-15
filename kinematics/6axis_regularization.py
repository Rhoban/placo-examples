import pinocchio
import placo
import numpy as np
import argparse
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, point_viz

"""
6axis robot following a 3D position target, to demostrate diferent regularization techniques

By default: the fallback regularization will apply, resulting in a minimization of delta_q
With --strong_l2: a strong L2 regularization is added, resulting in reduction of motion
With --posture: a posture regularization will be applied, with a target of 0 for all joints
With --kinetic: a kinetic energy regularization will be applied

(Depending on the robot configuration, the target may not be reachable)
"""

parser = argparse.ArgumentParser()
parser.add_argument("-k", "--kinetic", action="store_true")
parser.add_argument("-m", "--manipulability", action="store_true")
parser.add_argument("-p", "--posture", action="store_true")
parser.add_argument("-l", "--strong_l2", action="store_true")
args = parser.parse_args()

# Loading the robot
robot = placo.RobotWrapper("../models/6axis", placo.Flags.ignore_collisions)

# Creating the solver
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)

effector_task = solver.add_position_task("effector", np.zeros(3))
effector_task.configure("effector", "soft", 1.0)

if args.strong_l2:
    solver.add_regularization_task(1e2)
elif args.kinetic:
    solver.add_kinetic_energy_regularization_task(1e-6)
elif args.posture:
    posture = solver.add_joints_task()
    posture.set_joints({f"r{i}": 0.0 for i in range(1, 7)})
    posture.configure("posture", "soft", 1e-6)
elif args.manipulability:
    manipulability = solver.add_manipulability_task("effector", "both", 1.0)
    manipulability.configure("manipulability", "soft", 1e-3)

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
    t += dt

    effector_task.target_world = np.array([1.0, 0.5 * np.sin(t), 1.0])

    # Solving the IK
    solver.solve(True)
    robot.update_kinematics()

    # Displaying the robot, effector and target
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")
    point_viz("target", effector_task.target_world, radius=0.05)


run_loop()
