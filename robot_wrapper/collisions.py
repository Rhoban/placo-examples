import numpy as np
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, points_viz, get_viewer
import placo

robot = placo.RobotWrapper("../models/sigmaban/")
viz = robot_viz(robot)


t = 0
dt = 0.01


@schedule(interval=dt)
def loop():
    global t
    t += dt

    # Moving the robot leg
    robot.set_joint("left_hip_roll", np.sin(t) * 0.2)

    # Getting the collisions
    collisions = robot.self_collisions(False)

    # Printing the collisions
    print(f"Found {len(collisions)} collisions")
    for collision in collisions:
        print("- Collision betweek " + collision.bodyA + " and " + collision.bodyB)

    # Displaying the collisions
    points_viz(
        "collisions",
        np.array([collision.get_contact(0) for collision in collisions]),
        color=0xFF0000,
    )

    # Displaying the robot
    viz.display(robot.state.q)


run_loop()
