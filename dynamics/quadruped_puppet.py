import pinocchio
import numpy as np
import placo
from ischedule import schedule, run_loop
from placo_utils.tf import tf
from placo_utils.visualization import robot_viz, contacts_viz

"""
A quadruped is following tasks that is not physically feasible (mostly flying in the air).

The puppet contact is used, allowing arbitrary external forces to be added to make the tasks feasible.
"""

robot = placo.RobotWrapper("../models/quadruped/")

# Putting the robot in a standing position
robot.update_kinematics()
robot.set_T_world_frame("trunk", tf.translation_matrix([0.0, 0.0, 0.13]))
for k in range(1, 5):
    robot.set_joint(f"leg{k}_b", -0.5)
    robot.set_joint(f"leg{k}_c", 1.3)
robot.update_kinematics()

solver = placo.DynamicsSolver(robot)

T_world_trunk = tf.translation_matrix([0.0, 0.0, 0.05])
trunk_task = solver.add_frame_task("trunk", T_world_trunk)

# Adding contact points and tasks
leg_targets = {}
leg_tasks = {}
for k in range(1, 5):
    leg_targets[k] = robot.get_T_world_frame(f"leg{k}")[:3, 3]
    leg_world = robot.get_T_world_frame(f"leg{k}")[:3, 3]
    leg_world[2] = 0.0
    leg_tasks[k] = solver.add_position_task(f"leg{k}", leg_world)

solver.enable_joint_limits(True)

viz = robot_viz(robot)

# Adding puppet contact, enabling possible forces to be applied
contact = solver.add_puppet_contact()

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
        trunk_target = T_world_trunk[:3, 3] + np.array([0.0, 0.0, np.sin(t) * 0.03])
        trunk_task.position().target_world = trunk_target
        trunk_task.orientation().R_world_frame = tf.rotation_matrix(
            np.sin(t * 3.0) * 0.2, np.array([1.0, 0.0, 0.0])
        )[:3, :3]

        for k in range(1, 5):
            target = leg_targets[k] + np.array([0.0, 0.0, np.sin(t + k) * 0.03])
            leg_tasks[k].target_world = target

        solver.solve(True)
        robot.update_kinematics()

    viz.display(robot.state.q)
    contacts_viz(solver)


run_loop()
