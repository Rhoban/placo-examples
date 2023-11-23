import placo
import time
import argparse
from placo_utils.visualization import robot_viz, robot_frame_viz

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--robot", type=str, default="6axis")
args = arg_parser.parse_args()

robot = placo.RobotWrapper(args.robot)
viz = robot_viz(robot)

while True:
    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")
    time.sleep(1)
