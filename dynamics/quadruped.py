import pinocchio
import numpy as np
import placo
import argparse
from ischedule import schedule, run_loop
from placo_utils.tf import tf
from placo_utils.visualization import robot_viz, contacts_viz

"""
In this example, a quadruped is placed on the floor with four unilateral contact points.
The trunk is moving, following a circle in the x-y plane.

If you pass --weight_tangentials, extra weights will be added to weight the tangential forces.
As a result, the forces will be more normal to the contact surface.
"""

args_parser = argparse.ArgumentParser()
args_parser.add_argument("--weight_tangentials", action="store_true")
args = args_parser.parse_args()

robot = placo.RobotWrapper("../models/quadruped/")

# Putting the robot in a standing position
robot.update_kinematics()
robot.set_T_world_frame("trunk", tf.translation_matrix([0.0, 0.0, 0.13]))
for k in range(1, 5):
    robot.set_joint(f"leg{k}_b", -0.75)
    robot.set_joint(f"leg{k}_c", 1.8)
robot.update_kinematics()

solver = placo.DynamicsSolver(robot)

T_world_trunk = tf.translation_matrix([0.0, 0.0, 0.05])
trunk_task = solver.add_frame_task("trunk", T_world_trunk)

# Adding contact points and tasks
for k in range(1, 5):
    leg_world = robot.get_T_world_frame(f"leg{k}")[:3, 3]
    leg_world[2] = 0.0
    leg_task = solver.add_position_task(f"leg{k}", leg_world)
    leg_task.configure(f"leg{k}_pos", "soft", 1.0)
    leg_task.kp = 1e5
    contact = solver.add_unilateral_point_contact(leg_task)
    # Adding some cost to discourage tangential forces
    if args.weight_tangentials:
        contact.weight_tangentials = 1e-4

# Enabling torque and velocity limits
solver.enable_torque_limits(True)
solver.enable_velocity_limits(True)
solver.enable_joint_limits(True)

viz = robot_viz(robot)


t = 0
solver.dt = 0.001
view_fps = 50  # FPS for viewer
steps_per_view = int((1 / view_fps) / solver.dt)


@schedule(interval=(1 / view_fps))
def loop():
    global t

    for _ in range(steps_per_view):
        t += solver.dt

        # Moving the trunk
        trunk_target = (
            T_world_trunk[:3, 3] + np.array([np.cos(t), np.sin(t), 0.0]) * 0.05
        )
        trunk_task.position().target_world = trunk_target

        solver.solve(True)
        robot.update_kinematics()

    viz.display(robot.state.q)
    contacts_viz(solver)


run_loop()
