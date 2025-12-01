import time
import example
from placo_utils.visualization import robot_viz

example_solver = example.SolverExample()
robot = example_solver.get_robot()

viz = robot_viz(robot)
t = 0
dt = 0.02

while True:
    example_solver.update_trajectory(t)

    viz.display(robot.state.q)
    t += dt
    time.sleep(dt)