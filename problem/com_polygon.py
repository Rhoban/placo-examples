import placo
import numpy as np
import matplotlib.pyplot as plt

"""
From a given center of mass (CoM) position, this example finds the closest
point that is inside a given support polygon with a margin.
"""

# Polygon to fit in
polygon = [
    np.array([1.0, 1.0]),
    np.array([1.0, 2.0]),
    np.array([2.0, 1.0]),
]

# Margin (m)
margin = 0.1


def find_com_target(com_x, com_y):
    # Creating the problem
    problem = placo.Problem()
    xy = problem.add_variable(2)
    problem.add_constraint(
        placo.PolygonConstraint.in_polygon_xy(xy.expr(), polygon, margin)
    )

    # The final CoM should be as close as possible from the original com
    problem.add_constraint(xy.expr() == np.array([com_x, com_y])).configure("soft", 1.0)

    # Solving the QP problem
    problem.solve()

    return xy.value


# Plotting (animation)
t = 0

while True:
    # Current CoM position
    com_x, com_y = [1.3 + np.cos(t), 1.3 + np.sin(t)]

    target_x, target_y = find_com_target(com_x, com_y)

    plt.clf()
    plt.scatter(com_x, com_y, c="r", label="Original CoM")
    plt.scatter(target_x, target_y, c="b", label="New CoM")
    plt.plot(
        [p[0] for p in polygon + [polygon[0]]],
        [p[1] for p in polygon + [polygon[0]]],
        c="g",
        label="Support polygon",
    )
    plt.grid()
    plt.xlim(0, 3)
    plt.ylim(0, 3)
    plt.legend()
    plt.pause(0.1)
    t += 0.1
