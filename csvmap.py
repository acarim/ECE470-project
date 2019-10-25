import numpy as np
import pandas as pd


class Rectangle:
    """
        A rectangle contains the properties of each rectangle in the scene.
    """

    def __init__(self, x_center=None, y_center=None, x_dim=None, y_dim=None):
        self.x_center = x_center
        self.y_center = y_center
        self.x_dim = x_dim
        self.y_dim = y_dim


class Map:
    """
        A Map does this and this...
    """

    def __init__(self):
        self.x_dim = 20
        self.y_dim = 20
        self.n = 50
        self.map = np.zeros([self.n ** 2, 3])
        self.wall_thickness = 0.6
        self.corridor_width = 6
        self.cuboid_size = 0.6
        self.wall_height = 1
        self.n_sin = 50  # Sinusoidal track boundary
        self.rectangles = [Rectangle(x_center=0, y_center=0, x_dim=self.cuboid_size, y_dim=self.cuboid_size)]

        for x in np.linspace(-self.x_dim / 2, self.x_dim / 2, self.n_sin):
            for position in (-1, 1):
                self.rectangles.append(
                    Rectangle(
                        x_center=x,
                        y_center=5 * np.sin(x * 2 * np.pi / self.x_dim) + position * self.corridor_width * 0.5,
                        x_dim=self.wall_thickness,
                        y_dim=self.wall_thickness
                    )
                )

    @staticmethod
    def bound_compare(x, a, b):
        return min(a, b) < x < max(a, b)

    def create_map(self):
        X = np.linspace(-self.x_dim / 2, self.x_dim / 2, self.n)
        Y = np.linspace(-self.y_dim / 2, self.y_dim / 2, self.n)
        out = 0.001 * np.ones([self.n, self.n])
        counter = 0

        for i, x in enumerate(X):
            for j, y in enumerate(Y):
                x_within_bounds = self.bound_compare(x, -(0.5 * self.x_dim - self.wall_thickness),
                                                     (0.5 * self.x_dim - self.wall_thickness))
                y_within_bounds = self.bound_compare(y, -(0.5 * self.y_dim - self.wall_thickness),
                                                     (0.5 * self.y_dim - self.wall_thickness))
                if not x_within_bounds or not y_within_bounds:
                    out[i][j] = self.wall_height

                for r in self.rectangles:
                    x_within_bounds = self.bound_compare(x, r.x_center - 0.5 * r.x_dim, r.x_center + 0.5 * r.x_dim)
                    y_within_bounds = self.bound_compare(y, r.y_center - 0.5 * r.y_dim, r.y_center + 0.5 * r.y_dim)
                    if x_within_bounds and y_within_bounds:
                        out[i][j] = self.wall_height

                self.map[counter] = [x, y, out[i][j]]
                counter += 1
        return out


ourMap = Map()
csv = ourMap.create_map()
MapFile = ourMap.map
df = pd.DataFrame(csv)
df.to_csv('CSVmap.csv', header=False, index=False)
df = pd.DataFrame(MapFile)
df.to_csv('3Colmap.csv', header=False, index=False)
