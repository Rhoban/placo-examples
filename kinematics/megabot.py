import time
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import (
    robot_viz,
    robot_frame_viz,
    frame_viz,
)

robot = placo.RobotWrapper("../models/megabot/", placo.Flags.ignore_collisions)

# Updating joint limits and velocity limits
for leg in range(1, 5):
    for r in range(1, 9):
        robot.set_joint_limits(f"l{leg}_r{r}", -2 * np.pi, 2 * np.pi)
        robot.set_velocity_limit(f"l{leg}_r{r}", 1e8)
    for c in range(1, 4):
        # we set join at zero in onshape model before export
        security_margin = 0.02  # [m]
        robot.set_joint_limits(f"l{leg}_c{c}", security_margin, 0.2 - security_margin)

# Initializing the kinematics solver
solver = placo.KinematicsSolver(robot)

# Adding hard closing loop constraints to match l*_cl*_1 and l*_cl*_2 in the XZ plane
for leg in range(1, 5):
    for cl in range(1, 5):
        closing_loop = solver.add_relative_position_task(
            f"l{leg}_cl{cl}_1", f"l{leg}_cl{cl}_2", np.array([0.0, 0.0, 0.0])
        )
        closing_loop.configure(f"l{leg}_cl{cl}", "hard", 1.0)
        closing_loop.mask.set_axises("xz")

# Initializing T_world_base to identity
T_world_leg1 = robot.get_T_world_frame("leg_1")
z = T_world_leg1[2, 3]
T_world_base = robot.get_T_world_frame("base")
dz = T_world_base[2, 3] - T_world_leg1[2, 3]
T_world_base = np.eye(4)
T_world_base[2, 3] = 0.45
robot.set_T_world_frame("base", T_world_base)
robot.update_kinematics()

# We set the position of the base in the world
T_world_base = robot.get_T_world_frame("base")
base_task = solver.add_frame_task("base", T_world_base)
base_task.configure("base", "soft", 1.0, 10.0)

# We set the position of legs
legs_distance = 1.1
leg_targets = {
    "leg_1": np.array([legs_distance, -legs_distance, 0]),
    "leg_2": np.array([-legs_distance, -legs_distance, 0]),
    "leg_3": np.array([-legs_distance, legs_distance, 0]),
    "leg_4": np.array([legs_distance, legs_distance, 0]),
}
for leg in range(1, 5):
    name = f"leg_{leg}"
    T_world_leg = robot.get_T_world_frame(name)
    leg_task = solver.add_position_task(name, leg_targets[name])
    leg_task.configure(name, "hard", 1.0)

# Initializing the viewer
viz = robot_viz(robot)
t: float = 0.0
dt: float = 0.01


@schedule(interval=dt)
def loop():
    global t
    t += dt

    # Update the body target frame
    T = T_world_base.copy()
    T[0, 3] = np.cos(t * 2) * 0.15
    T[1, 3] = np.sin(t * 2) * 0.15
    base_task.T_world_frame = T

    # Updating the kinematics and solving the IK
    robot.update_kinematics()
    solver.solve(True)

    # Displaying
    viz.display(robot.state.q)
    robot_frame_viz(robot, "base")
    frame_viz("base_target", T, 0.5)


run_loop()
