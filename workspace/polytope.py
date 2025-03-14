import pickle
import placo
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import linprog
from scipy.spatial import ConvexHull, HalfspaceIntersection
from matplotlib.colors import LightSource
import mpl_toolkits.mplot3d as a3


class Polytope:
    """
    A 3D Polytope of the form Ax <= b
    """

    def __init__(self, A, b):
        self.A = np.array(A)
        self.b = np.array(b)

    def save(self, filename: str):
        """
        Saves the Polytope to a pickle file
        """
        pickle.dump(self, open(filename, "wb"))

    def load(filename: str):
        """
        Loads the Polytope from a pickle file
        """
        return pickle.load(open(filename, "rb"))

    def find_closest(self, point: np.ndarray):
        """
        Find the closest point on the polytope from a given point
        """
        problem = placo.Problem()
        position = problem.add_variable(self.A.shape[1])

        problem.add_constraint(position.expr() == point).configure("soft", 1.0)
        problem.add_constraint(position.expr().left_multiply(self.A) <= self.b)
        problem.solve()

        return position.value

    def simplify(self):
        """
        Simplifies the polytope, by removing redundant equations
        """
        line = 0
        original_lines = len(self.A)
        while line < len(self.A):
            direction = self.A[line]

            A_tmp = list(self.A.copy())
            b_tmp = list(self.b.copy())
            del A_tmp[line]
            del b_tmp[line]
            A_tmp = np.array(A_tmp)
            b_tmp = np.array(b_tmp)

            x_1 = linprog(-direction, A_ub=A_tmp, b_ub=b_tmp, bounds=(None, None)).x

            test = (self.A @ x_1) < self.b

            if test[line]:
                self.A = A_tmp
                self.b = b_tmp
            else:
                line += 1

        print(f"* Constraints before: {original_lines}, after: {len(self.A)}")

    def show(self, show_points: bool = False) -> None:
        """
        Showing the Polytope using half-plane intersections, and optionally the original points
        """
        M = np.hstack((self.A, -self.b.reshape(-1, 1)))
        hd = HalfspaceIntersection(M, np.array([0.0, 0.0, 0.0]))
        hull = ConvexHull(hd.intersections)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        for i in hull.simplices:
            plt.plot(
                hd.intersections[i, 0],
                hd.intersections[i, 1],
                hd.intersections[i, 2],
                "g-",
            )

        if show_points:
            points = np.array([x * y for x, y in zip(self.A, self.b)])
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], "b")

        plt.show()
