import placo
import numpy as np

"""
From a given center of mass (CoM) position, this example finds the closest
point that is inside a given support polygon with a margin.
"""

# Current CoM position
com_x, com_y = [0.0, 0.0]

# Polygon to fit in
polygon = [
    np.array([1.0, 1.0]),
    np.array([1.0, 2.0]),
    np.array([2.0, 1.0]),
]

# Margin (m)
margin = 0.1

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

# Solution
print(xy.value)
