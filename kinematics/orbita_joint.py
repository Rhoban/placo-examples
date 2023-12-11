import placo
import time
import numpy as np
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz

"""
Orbita (3 axis rotation with loop closure constraint)
Controlled with joints-space orders
"""

robot = placo.RobotWrapper("../models/orbita/", placo.Flags.ignore_collisions)
robot.set_T_world_fbase(np.eye(4))
viz = robot_viz(robot)

solver = robot.make_solver()
solver.mask_fbase(True)
solver.enable_joint_limits(False)
solver.enable_velocity_limits(True)

# Adding closure constraints
for closing in ["closing_ring2_br2", "closing_ring3_br2"]:
    task = solver.add_relative_position_task(
        f"{closing}_1", f"{closing}_2", np.array([0, 0, 0])
    )
    task.configure(closing, "hard")
    task.mask.set_axises("xy")

# Creating a task to control joint
joints_task = solver.add_joints_task()
joints_task.set_joints(
    {
        "ring1": 0,
        "ring2": 0,
        "ring3": 0,
    }
)

dt = 0.01
solver.dt = dt
t = 0

while True:
    # Updating the task's target
    joints_task.set_joints({"ring1": np.sin(t * 3)})

    # Solving kinematics
    robot.update_kinematics()
    solver.solve(True)
    solver.dump_status()

    # Displaying
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")

    t += dt
    time.sleep(dt)
