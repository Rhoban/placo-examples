import time
import placo
import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import (
    robot_viz,
    robot_frame_viz,
    frame_viz,
    contacts_viz,
)

robot = placo.RobotWrapper("../models/megabot/", placo.Flags.ignore_collisions)

viz = robot_viz(robot)

# Updating joint limits and velocity limits
for leg in range(1, 5):
    for r in range(1, 9):
        # Passive joints have no limits and velocity limits
        robot.set_joint_limits(f"l{leg}_r{r}", -2 * np.pi, 2 * np.pi)
        robot.set_velocity_limit(f"l{leg}_r{r}", 1e8)
    for c in range(1, 4):
        # The cylinder has a stroke of 20 cm, we set a limit of 1kN and 0.2 m/s
        security_margin = 0.02  # [m]
        robot.set_joint_limits(f"l{leg}_c{c}", security_margin, 0.2 - security_margin)
        robot.set_torque_limit(f"l{leg}_c{c}", 1000.0)
        robot.set_velocity_limit(f"l{leg}_c{c}", 0.2)

# Initializing the dynamics solver
solver = placo.DynamicsSolver(robot)

# We reduce dramatically the cost of torque, since Megabot is involving huge forces
solver.torque_cost = 1e-6

# Adding hard closing loop constraints to match l*_cl*_1 and l*_cl*_2 in the XZ plane
for leg in range(1, 5):
    for cl in range(1, 5):
        closing_loop = solver.add_relative_position_task(
            f"l{leg}_cl{cl}_1", f"l{leg}_cl{cl}_2", np.array([0.0, 0.0, 0.0])
        )
        closing_loop.configure(f"l{leg}_cl{cl}", "hard", 1.0)
        closing_loop.mask.set_axises("xz")
        contact = solver.add_task_contact(closing_loop)

# Enforcing passive DoFs to have zero torque
torque_task = solver.add_torque_task()
for leg in range(1, 5):
    for r in range(1, 9):
        torque_task.set_torque(f"l{leg}_r{r}", 0.0, 0.0, 1.0)
torque_task.configure("passive", "hard")

# Initializing T_world_base to identity
T_world_leg1 = robot.get_T_world_frame("leg_1")
z = T_world_leg1[2, 3]
T_world_base = robot.get_T_world_frame("base")
dz = T_world_base[2, 3] - T_world_leg1[2, 3]
T_world_base = np.eye(4)
T_world_base[2, 3] = 0.4
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
    leg_contact = solver.add_point_contact(leg_task)

t = 0
solver.dt = 0.005
view_fps = 25  # FPS for viewer
steps_per_view = int((1 / view_fps) / solver.dt)

# Adding a puppet contact for initialization
puppet_contact = solver.add_puppet_contact()
for k in range(2000):
    solver.solve(True)
    robot.update_kinematics()
solver.remove_contact(puppet_contact)

solver.enable_torque_limits(True)
solver.enable_velocity_limits(True)
solver.enable_joint_limits(True)

@schedule(interval=(1 / view_fps))
def loop():
    global t

    for _ in range(steps_per_view):
        t += solver.dt

        # Update the body target frame
        T = T_world_base.copy()
        T[0, 3] = np.cos(t) * 0.15
        T[1, 3] = np.sin(t) * 0.15
        base_task.T_world_frame = T

        base_task.position().dtarget_world = np.array(
            [-np.sin(t) * 0.15, np.cos(t) * 0.15, 0.0]
        )

        solver.solve(True)
        robot.update_kinematics()

    # Displaying
    viz.display(robot.state.q)
    robot_frame_viz(robot, "base")
    contacts_viz(solver, ratio=1e-3, radius=0.03)
    frame_viz("target", base_task.T_world_frame, 0.5)


run_loop()
