import numpy as np
import math
import roboticstoolbox as rtb


class DeltaRobot:
    def __init__(self, r, h, s, k, Zt):
        self.r = r
        self.h = h
        self.s = s
        self.k = k
        self.Zt = Zt

    def set_joint_angles(self, J1, J2, J3):
        self.J1 = J1
        self.J2 = J2
        self.J3 = J3
        self.c = np.cos(np.radians([J1, J2, J3]))
        self.sin_J = np.sin(np.radians([J1, J2, J3]))

    def calculate_bicep_endpoints(self):
        self.B1 = np.array([self.r + self.h * self.c[0], 0, -self.h * self.sin_J[0]])
        self.B2 = np.array([-(self.r + self.h * self.c[1]) / 2, np.sqrt(3) * (self.r + self.h * self.c[1]) / 2,
                            -self.h * self.sin_J[1]])
        self.B3 = np.array([-(self.r + self.h * self.c[2]) / 2, -np.sqrt(3) * (self.r + self.h * self.c[2]) / 2,
                            -self.h * self.sin_J[2]])

    def calculate_centers_of_spheres(self):
        self.P1 = np.array([self.r + self.h * self.c[0] - self.s, 0, -self.h * self.sin_J[0]])
        self.P2 = np.array(
            [-(self.r + self.h * self.c[1] - self.s) / 2, np.sqrt(3) * (self.r + self.h * self.c[1] - self.s) / 2,
             -self.h * self.sin_J[1]])
        self.P3 = np.array(
            [-(self.r + self.h * self.c[2] - self.s) / 2, -np.sqrt(3) * (self.r + self.h * self.c[2] - self.s) / 2,
             -self.h * self.sin_J[2]])

    def calculate_intersection(self):
        U = self.P2 - self.P1
        V = self.P3 - self.P1

        x_hat = U / np.linalg.norm(U)
        l = np.dot(x_hat, V)

        y_hat = (V - np.dot(l, x_hat)) / np.linalg.norm(V - np.dot(l, x_hat))
        h = np.dot(y_hat, V)

        z_hat = np.cross(x_hat, y_hat)

        x = np.linalg.norm(U) ** 2 / (2 * np.linalg.norm(U))
        y = (l ** 2 + h ** 2 - 2 * l * x) / (2 * h)
        z_squared = self.k ** 2 - x ** 2 - y ** 2

        if z_squared < 0:
            raise ValueError(
                "The square root of a negative number is not a real number. Perhaps the robot dimensions are invalid")

        z = - np.sqrt(z_squared)

        self.WP = self.P1 + x * x_hat + y * y_hat + z * z_hat
        self.TCP = self.WP + self.Zt * z_hat

    def calculate_kinematics(self, J1, J2, J3):
        self.set_joint_angles(J1, J2, J3)
        self.calculate_bicep_endpoints()
        self.calculate_centers_of_spheres()
        self.calculate_intersection()
        return np.around(self.TCP, 2)
    