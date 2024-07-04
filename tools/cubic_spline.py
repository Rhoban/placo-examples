import placo
import numpy as np
import matplotlib.pyplot as plt

"""
Build a cubic spline and plot its position and velocity.
"""

spline = placo.CubicSpline()

# Arguments are time, position and velocity
spline.add_point(0.0, 0.0, 0.0)
spline.add_point(1.0, 3.0, 0.0)
spline.add_point(2.0, -2.0, 0.0)
spline.add_point(3.0, 0.0, 0.0)


ts = np.linspace(0, 3, 100)
positions = np.array([spline.pos(t) for t in ts])
velocities = np.array([spline.vel(t) for t in ts])

plt.plot(ts, positions, label="position")
plt.plot(ts, velocities, label="velocity")
plt.grid()
plt.legend()
plt.show()
