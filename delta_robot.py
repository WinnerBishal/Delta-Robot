import numpy as np
from scipy.spatial import distance
import math
import roboticstoolbox as rtb
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import warnings
import plotly.graph_objects as go


class DeltaRobot:
    def __init__(self, r, h, s, k, Zt):
        self.r = r
        self.h = h
        self.s = s
        self.k = k
        self.Zt = Zt
        self.X_upper = 600
        self.X_lower = -300
        self.Y_upper = 250
        self.Y_lower = -250
        self.Z_upper = -180
        self.Z_lower = -550
        
    # FORWARD KINEMATICS
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

    def calculate_fwd_kinematics(self, J1, J2, J3):
        self.set_joint_angles(J1, J2, J3)
        self.calculate_bicep_endpoints()
        self.calculate_centers_of_spheres()
        self.calculate_intersection()
        #Validate Limits
        if self.TCP[0] > self.X_upper or self.TCP[0] < self.X_lower:
            print("X value is out of range")
            return
        if self.TCP[1] > self.Y_upper or self.TCP[1] < self.Y_lower:
            print("Y value is out of range")
            return
        if self.TCP[2] > self.Z_upper or self.TCP[2] < self.Z_lower:
            print("Z value is out of range")
            return
        
        return np.around(self.TCP, 2)
    
    # INVERSE KINEMATICS
    def set_TCP(self, X_in, Y_in, Z_in):
        self.WP_1 = np.array([X_in, Y_in, Z_in + self.Zt])
        self.pos_rot = rtb.ET.Rz(-120, 'degrees')
        self.pos_rot = self.pos_rot.A()[:3, :3]
        
        self.WP_2 = np.matmul(self.pos_rot, self.WP_1)

        self.neg_rot = rtb.ET.Rz(120, 'degrees')
        self.neg_rot = self.neg_rot.A()[:3, :3]

        self.WP_3 = np.matmul(self.neg_rot, self.WP_1)
        
        self.WP_s = [self.WP_1, self.WP_2, self.WP_3]
        
    def intersect_circles(self, x1, y1, r1, x2, y2, r2):
        dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

        if dist > r1 + r2 or dist < abs(r1 - r2) or dist == 0:
            return 

        a = (r1**2 - r2**2 + dist**2) / (2 * dist)
        h = math.sqrt(r1**2 - a**2)

        x3 = x1 + a * (x2 - x1) / dist
        y3 = y1 + a * (y2 - y1) / dist

        intersection1 = (x3 + h * (y2 - y1) / dist, y3 - h * (x2 - x1) / dist)
        intersection2 = (x3 - h * (y2 - y1) / dist, y3 + h * (x2 - x1) / dist)

        return intersection1 if intersection1[0] > intersection2[0] else intersection2
    
    def solve_circles(self):
        # Solving the intersecting circles
        self.solved_joints = []
        
        for i in range(3):
            x1 = self.r
            y1 = 0
            
            x2 = self.WP_s[i][0] + self.s
            y2 = self.WP_s[i][2]
            
            r1 = self.h
            r2 = np.sqrt(self.k**2 - self.WP_s[i][1]**2)
            
            x3, y3 = self.intersect_circles(x1, y1, r1, x2, y2, r2)
            joint_angle = np.degrees(np.arctan2(y3, x3))
            self.solved_joints.append(round(-(1.66021)*joint_angle, 2))
            
        # for i in range(3):
        #     def equations(vars):
        #         B_x = vars[0]
        #         B_z = vars[1]
        #         eq1 = (B_x - self.r)**2 + B_z**2 - self.h**2
        #         eq2 = (B_x - self.WP_s[i][0] - self.s)**2 + (B_z-self.WP_s[i][2])**2 - self.k**2 + self.WP_s[i][1]**2
        #         return np.array([eq1, eq2]) 
              
        #     initial_guess = [self.r+self.h, 0]
        #     try:
        #         roots = fsolve(equations, initial_guess)
        #         joint_angle = np.degrees(np.arctan2(float(roots[1]), float(roots[0])))
        #         self.solved_joints.append(round(-joint_angle, 2))
        #         # -(1.66021)*
        #     except Exception as e:
        #         warnings.warn(f"Failed to solve the system of equations: {e}")

    def calculate_inv_kinematics(self, X_in, Y_in, Z_in):
        # Validate Limits
        if X_in > self.X_upper or X_in < self.X_lower:
            print("X value is out of range")
            return
        if Y_in > self.Y_upper or Y_in < self.Y_lower:
            print("Y value is out of range")
            return
        if Z_in > self.Z_upper or Z_in < self.Z_lower:
            print("Z value is out of range")
            return
        
        self.set_TCP(X_in, Y_in, Z_in)
        self.solve_circles()
        return self.solved_joints
    

def plot_delta_robot(B1, B2, B3, WP, r, s):
    fig = go.Figure(layout = go.Layout(height = 600, autosize = True))

    theta = np.linspace(0, 2*np.pi, 100)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.zeros_like(theta)  

    fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', line=dict(color='red')))

    fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], mode='markers', marker=dict(color='red', size=5)))

    base_joints = []
    Bs = [B1, B2, B3]
    for i in range(3):
        angle_rad = 2 * np.pi * i / 3  
        joint = [r * np.cos(angle_rad), r * np.sin(angle_rad), 0]
        base_joints.append(joint)
        fig.add_trace(go.Scatter3d(x=[joint[0]], y=[joint[1]], z=[joint[2]], mode='markers', marker=dict(color='blue', size=3)))  # plot joint

        
        fig.add_trace(go.Scatter3d(x=[joint[0], Bs[i][0]], y=[joint[1], Bs[i][1]], z=[joint[2], Bs[i][2]], mode='lines', line=dict(color='green')))

    # Generate coordinates for end effector circle
    x_ee = WP[0] + s * np.cos(theta)
    y_ee = WP[1] + s * np.sin(theta)
    z_ee = WP[2] * np.ones_like(theta)

    fig.add_trace(go.Scatter3d(x=x_ee, y=y_ee, z=z_ee, mode='lines', line=dict(color='red')))

    fig.add_trace(go.Scatter3d(x=[WP[0]], y=[WP[1]], z=[WP[2]], mode='markers', marker=dict(color='blue', size=3)))

    # Plot end effector joints
    end_effector_joints = []
    for i in range(3):
        angle_rad = 2 * np.pi * i / 3  
        joint_ee = [WP[0] + s * np.cos(angle_rad), WP[1] + s * np.sin(angle_rad), WP[2]]
        end_effector_joints.append(joint_ee)
        fig.add_trace(go.Scatter3d(x=[joint_ee[0]], y=[joint_ee[1]], z=[joint_ee[2]], mode='markers', marker=dict(color='blue', size=3)))  # plot joint

        
        fig.add_trace(go.Scatter3d(x=[Bs[i][0], joint_ee[0]], y=[Bs[i][1], joint_ee[1]], z=[Bs[i][2], joint_ee[2]], mode='lines', line=dict(color='purple')))

    fig.update_layout(scene=dict(
        xaxis_title='X',
        yaxis_title='Y',
        zaxis_title='Z',
        aspectratio=dict(x=1, y=1, z=1),
        camera=dict(
            up=dict(x=0, y=0, z=1),
            center=dict(x=0, y=0, z=0),
            eye=dict(x=1.5, y=1.5, z=0.5)
        ),
        xaxis=dict(range=[-500, 500]),
        yaxis=dict(range=[-500, 500]),
        zaxis=dict(range=[-600, 200]))
    )

    return fig




