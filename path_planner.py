import numpy as np
import plotly.graph_objects as go
from multi_page_ui import send_command

class PathPlanner:
    def __init__(self, P0, P1, P2, P3, robot):
        self.P0 = P0
        self.P1 = P1
        self.P2 = P2 
        self.P3 = P3
        self.robot = robot                         # Expects class DeltaRobot with all robot parameters
        self.t = np.linspace(0, 1, 100)

    def lerp(self, start_p, end_p, t):
        start_p = np.array(start_p)
        end_p = np.array(end_p)
        return (1 - t) * start_p + t * end_p
    
    def generate_spline(self):
        # start_point, cp_1, cp_2, end_point are all 3D points
        # return a list of 3D points
        self.spline = []
        for t in self.t:
            A = self.lerp(self.P0, self.P1, t)
            B = self.lerp(self.P1, self.P2, t)
            C = self.lerp(self.P2, self.P3, t)
            D = self.lerp(A, B, t)
            E = self.lerp(B, C, t)
            F = self.lerp(D, E, t)
            self.spline.append(F)
        
        return self.spline
    
    def get_point_at_spline(self, t_value):
        point = self.P0*(-t_value**3+ 3*t_value**2-3*t_value+1) + self.P1*(3*t_value**3-6*t_value**2+3*t_value) + self.P2*(-3*t_value**3+3*t_value**2) + self.P3*(t_value**3)
        return point
    
    def plot_spline(self, robot_fig):
        # plot the spline
        x, y, z = map(list, zip(*self.spline))
        fig = go.Figure(robot_fig)
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', name='Spline Path', line = dict(width = 7, dash = 'dashdot')))
        fig.update_layout(title='Designated Spline Path', scene=dict(xaxis_title='X', yaxis_title='Y', zaxis_title='Z'))
        return fig
        
    def spline_to_joints(self):
        # convert the spline to joint angles
        self.joint_angles_for_spline = []
        for i in range(len(self.spline)):
            self.joint_angles_for_spline.append(self.robot.calculate_inv_kinematics(self.spline[i][0], self.spline[i][1], self.spline[i][2]))
        
        return self.joint_angles_for_spline
    
    def plot_spline_joint_space(self):
        # plot the joint angles
        joint_1, joint_2, joint_3 = map(list, zip(*self.joint_angles_for_spline))
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=self.t, y=joint_1, mode='lines', name='Joint 1'))
        fig.add_trace(go.Scatter(x=self.t, y=joint_2, mode='lines', name='Joint 2'))
        fig.add_trace(go.Scatter(x=self.t, y=joint_3, mode='lines', name='Joint 3'))
        fig.update_layout(title='Joint Angles vs. parameter t', xaxis_title='parameter t', yaxis_title='Joint Angles')
        return fig

    def perform_spline_planning(self):
        # generate the spline
        self.generate_spline()

        # convert the spline to joint angles
        self.spline_to_joints()

    def generate_line(self):
        # Interpolate line between P0 and P3
        self.line = []
        for t in self.t:
            self.line.append(self.lerp(self.P0, self.P3, t))
            
        return self.line
    
    def plot_line(self, robot_fig):
        # Interpolate line between P0 and P3
        x, y, z = map(list, zip(*self.line))
        fig = go.Figure(robot_fig)
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', name='Line Path'))
        return fig
    
    def line_to_joints(self):
        # convert the line to joint angles
        self.joint_angles_for_line = []
        for i in range(len(self.line)):
            self.joint_angles_for_line.append(self.robot.calculate_inv_kinematics(self.line[i][0], self.line[i][1], self.line[i][2]))
        
        return self.joint_angles_for_line
    
    def plot_line_joint_space(self):
        # plot the joint angles
        joint_1, joint_2, joint_3 = map(list, zip(*self.joint_angles_for_line))
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=self.t, y=joint_1, mode='lines', name='Joint 1'))
        fig.add_trace(go.Scatter(x=self.t, y=joint_2, mode='lines', name='Joint 2'))
        fig.add_trace(go.Scatter(x=self.t, y=joint_3, mode='lines', name='Joint 3'))
        fig.update_layout(title='Joint Angles vs. parameter t', xaxis_title='parameter t', yaxis_title='Joint Angles')
        return fig
    
    def perform_line_planning(self):
        # generate the line
        self.generate_line()

        # convert the line to joint angles
        self.line_to_joints()
