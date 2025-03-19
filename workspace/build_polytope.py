import numpy as np
import random
import tqdm
import placo
from polytope import Polytope
import matplotlib.pyplot as plt


class PointCloud:
    def __init__(self, points):
        self.points = points
        self.normals = points.copy()
        self.normals /= np.linalg.norm(self.normals, axis=1).reshape(-1, 1)
        self.init_b = None

    def build_polytope(self):
        A = self.normals.copy()
        b = self.normals.reshape(-1, 1, 3) @ self.points.reshape(-1, 3, 1)
        b = b.reshape(-1)

        return Polytope(A, b)

    def optimize_normals(self) -> bool:
        for k in tqdm.tqdm(range(len(self.points))):
            if len(self.points) > 256:
                sample_points = np.array(random.choices(self.points, k=256))
            else:
                sample_points = self.points
            point = self.points[k]
            point_further = point + 1e-3 * point / np.linalg.norm(point)
            problem = placo.Problem()
            normal = problem.add_variable(3)
            M = sample_points - point_further
            problem.add_constraint(normal.expr().left_multiply(M) <= -1).configure(
                "soft", 1
            )
            problem.solve()

            new_normal = normal.value / np.linalg.norm(normal.value)
            self.normals[k] = new_normal


import pickle

points = np.array(pickle.load(open("points.pkl", "rb")))

points = np.array(points)

points_cloud = PointCloud(points)

print(f"Optimizing point normals")
points_cloud.optimize_normals()

polytope = points_cloud.build_polytope()

print(f"Simplifying polytope")
polytope.simplify()
polytope.show(show_points=True)
polytope.save("workspace.pkl")
