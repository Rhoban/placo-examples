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
    np.array([0.0, -1.0]),
    np.array([-1.0, 2.0]),
]

# Margin (m)
margin = 0.1


def find_com_target(com_x, com_y, center_x=0.0, center_y=0.0):
    # Creating the problem
    problem = placo.Problem()
    ratio = problem.add_variable(1)

    # xy = problem.add_variable(2)
    xy = (ratio.expr() * (com_x - center_x) + center_x) / (
        ratio.expr() * (com_y - center_y) + center_y
    )

    problem.add_constraint(placo.PolygonConstraint.in_polygon_xy(xy, polygon, margin))
    problem.add_constraint(ratio.expr() == 1.0).configure("soft", 1.0)

    # Solving the QP problem
    problem.solve()

    return np.array([com_x - center_x, com_y - center_y]) * ratio.value + np.array(
        [center_x, center_y]
    )


# Plotting (animation)
t = 0

while True:
    # Current CoM position
    com_x, com_y = [1.3 + np.cos(t), 1.3 + np.sin(t)]

    # Point aimed at
    center_x, center_y = 0.0, 1.0

    target_x, target_y = find_com_target(com_x, com_y, center_x, center_y)

    plt.clf()
    plt.scatter(com_x, com_y, c="r", label="Original CoM")
    plt.scatter(target_x, target_y, c="b", label="New CoM")
    plt.plot(
        [p[0] for p in polygon + [polygon[0]]],
        [p[1] for p in polygon + [polygon[0]]],
        c="g",
        label="Support polygon",
    )
    plt.scatter(center_x, center_y, c="k", label="Center")
    plt.plot(
        [center_x, com_x], [center_y, com_y], c="b", label="Support polygon", ls="--"
    )
    plt.grid()
    plt.xlim(-3, 3)
    plt.ylim(-3, 3)
    plt.legend()
    plt.pause(0.1)
    t += 0.1
