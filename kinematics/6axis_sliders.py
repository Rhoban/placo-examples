import placo
from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, robot_frame_viz, joints_sliders

"""
6axis robot whose joints are controlled directly with sliders in the viewer (forward kinematics, no solver)
"""

# Loading the robot
robot = placo.RobotWrapper("../models/6axis", placo.Flags.ignore_collisions)

viz = robot_viz(robot)

# A slider for each joint, within its limits
sliders = joints_sliders(robot)


@schedule(interval=0.02)
def loop():
    # Joints values from the sliders
    for joint, slider in sliders.items():
        robot.set_joint(joint, slider.value)
    robot.update_kinematics()

    # Displaying the robot and effector
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")


run_loop()
