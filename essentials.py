import cv2, math, time, warnings
import numpy as np
import plotly.graph_objects as go
import numpy as np
from scipy.spatial import distance
import math
import roboticstoolbox as rtb
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import warnings
import plotly.graph_objects as go
from ultralytics import YOLO
import seaborn as sns
import modern_robotics as mr

'''
List of available elements:
ConveyorRobot Class
PathPlanner Class
send_command Function
image2Dworld3D Function
DeltaRobot Class
plot_delta_robot Function
ImageProcessor Class
adjust_for_conveyor_movement Function
GetObjects Function
PNPTestPath Class
'''
def adjust_for_conveyor_movement(object_point, time_to_reach, conveyor_speed_y):
    adjusted_point = object_point.copy()
    adjusted_point[1] += conveyor_speed_y * time_to_reach
    return adjusted_point

class ConveyorRobot:
    def __init__(self, robot, z_offset=10):
        self.robot = robot
        self.z_offset = z_offset

    def create_spline_path(self, start_point, end_point):
        start_point_offset = start_point.copy()
        start_point_offset[2] += self.z_offset

        end_point_offset = end_point.copy()
        end_point_offset[2] += self.z_offset

        path_planner = PathPlanner(start_point, start_point_offset, end_point_offset, end_point, self.robot)
        return path_planner.generate_spline()

    def process_batch(self, object_points, box_points):
        if len(object_points) != len(box_points):
            raise ValueError("Number of objects and boxes must be the same")

        full_path = []
        for i in range(len(object_points)):
            pick_path = self.create_spline_path(object_points[i], box_points[i])
            full_path.extend(pick_path)

            if i < len(object_points) - 1:
                transition_path = self.create_spline_path(box_points[i], object_points[i + 1])
                full_path.extend(transition_path)

        return full_path
    
def image2Dworld3D(im_P):
    robot_coords = np.array([[-379.443, 92.0], [-466.443,   97.0], [-384.443,  -48.0  ], [-473.443,  -42.0]])  
    image_coords = np.array([[500, 300], [500, 100], [200, 300], [200, 100]])  

    H, _ = cv2.findHomography(image_coords, robot_coords)
    
    point_img = np.array(im_P, dtype=np.float32)
    point_img = np.append(point_img, 1)
    point_world = np.dot(H, point_img)  

    point_world /= point_world[2]
    point_world = point_world[:2]
    
    z_constant = -520
    point_world = np.append(point_world, z_constant)
    
    return point_world

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
        
def send_command(ser, calibrate_motor=None, m1=None, m2=None, m3=None, gripper=None):
    command_parts = []
    
    # Calibration command
    if calibrate_motor is not None:
        command_parts.append(f"C{calibrate_motor}:1")
    
    # Movement commands
    if m1 is not None:
        command_parts.append(f"M1:{m1}")
    if m2 is not None:
        command_parts.append(f"M2:{m2}")
    if m3 is not None:
        command_parts.append(f"M3:{m3}")
    
    # Gripper command
    if gripper is not None:
        command_parts.append(f"G:{gripper}")
    
    # Construct the full command string
    full_command = ';'.join(command_parts) + '\n'
    
    # Send the command to the Arduino
    ser.write(full_command.encode())
    
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().strip()
            # print(f"Arduino says: {response}")
            break
    
class DeltaRobot:
    def __init__(self, r, h, s, k, Zt):
        self.r = r
        self.h = h
        self.s = s
        self.k = k
        self.Zt = Zt
        self.X_upper = 800
        self.X_lower = -500
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
        
        return np.around(self.TCP, 4)
    
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
    
    def solve_circles(self):
        self.solved_joints= []
        for i in range(3):
            def equations(vars):
                B_x = vars[0]
                B_z = vars[1]
                eq1 = (B_x - self.r)**2 + B_z**2 - self.h**2
                eq2 = (B_x - self.WP_s[i][0] - self.s)**2 + (B_z-self.WP_s[i][2])**2 - self.k**2 + self.WP_s[i][1]**2
                return np.array([eq1, eq2]) 
              
            initial_guess = [self.r+self.h, 0]
            try:
                roots = fsolve(equations, initial_guess)
                joint_angle = np.degrees(np.arctan2(float(roots[1]), float(roots[0]) - self.r))
                self.solved_joints.append(round(joint_angle, 4))
                # -(1.66021)*
            except Exception as e:
                warnings.warn(f"Failed to solve the system of equations: {e}")

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

    fig.add_trace(go.Scatter3d(x = x, y = y, z = z, mode = 'lines', line = dict(color = 'red')))

    fig.add_trace(go.Scatter3d(x = [0], y = [0], z = [0], mode = 'markers', marker = dict(color='red', size=5)))

    base_joints = []
    Bs = [B1, B2, B3]
    for i in range(3):
        angle_rad = 2 * np.pi * i / 3  
        joint = [r * np.cos(angle_rad), r * np.sin(angle_rad), 0]
        base_joints.append(joint)
        fig.add_trace(go.Scatter3d(x = [joint[0]], y = [joint[1]], z = [joint[2]], mode = 'markers', marker = dict(color='blue', size = 3)))  # plot joint

        
        fig.add_trace(go.Scatter3d(x = [joint[0], Bs[i][0]], y = [joint[1], Bs[i][1]], z = [joint[2], Bs[i][2]], mode = 'lines', line = dict(color = 'green')))

    # Generate coordinates for end effector circle
    x_ee = WP[0] + s * np.cos(theta)
    y_ee = WP[1] + s * np.sin(theta)
    z_ee = WP[2] * np.ones_like(theta)

    fig.add_trace(go.Scatter3d(x = x_ee, y = y_ee, z = z_ee, mode='lines', line = dict(color='red')))

    fig.add_trace(go.Scatter3d(x = [WP[0]], y = [WP[1]], z = [WP[2]], mode='markers', marker = dict(color = 'blue', size = 3)))

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

class ImageProcessor:
    def __init__(self, bgr_image):
        self.img = bgr_image.copy()
        self.org_img = self.img.copy()
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.cropped_objects = []
        self.processing_steps_images = []
        self.processing_steps_labels = ["Original Image", "Mask Image", "Distance Transform", "Foreground Image"]
        self.all_cnts = []  # Store detected contours
        self.cropped_objects_info = []  # Store cropped objects info
        
    def create_mask(self):
        lower = np.array([70, 70, 70])
        upper = np.array([255, 255, 255])
        mask = cv2.bitwise_not(cv2.inRange(self.img, lower, upper))

        # Filling holes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            cv2.drawContours(mask, [contour], 0, 255, -1)  # 255 is color, -1 means fill the contour  
        return mask
    
    def resize_with_padding(self, img, target_size=(128, 128), fill_value=0):
        """
        Resize the input image to the target size, maintaining the original aspect ratio by padding.
        """
        h, w = img.shape[:2]
        ratio = min(target_size[0] / h, target_size[1] / w)
        new_size = (int(w * ratio), int(h * ratio))
        resized_img = cv2.resize(img, new_size)

        delta_w = target_size[1] - new_size[0]
        delta_h = target_size[0] - new_size[1]
        top, bottom = delta_h // 2, delta_h - (delta_h // 2)
        left, right = delta_w // 2, delta_w - (delta_w // 2)

        padded_img = cv2.copyMakeBorder(resized_img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=fill_value)
        return padded_img
    
    def display_results(self):
        for i, cropped in enumerate(self.cropped_objects):
            plt.subplot(1, len(self.cropped_objects), i + 1)
            plt.imshow(cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB))
            plt.title(f"Cropped Object {i + 1}")
            plt.axis('off')
        plt.show()
    
    def display_masks(self):
        fig, ax = plt.subplots(1, len(self.processing_steps_images), figsize=(20, 20))
        for i, img in enumerate(self.processing_steps_images):
            if i == 0:
                ax[i].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            else:
                ax[i].imshow(img, cmap='gray')
            ax[i].set_title(self.processing_steps_labels[i])
            ax[i].axis('off')

    def apply_watershed(self):
        self.processing_steps_images.append(self.img)
        thresh = self.create_mask()
        self.processing_steps_images.append(thresh)

        sure_bg = cv2.dilate(thresh, np.ones((5, 5), np.uint8), iterations=1)

        dist_transform = cv2.distanceTransform(thresh, cv2.DIST_L2, 5)
        _, sure_fg = cv2.threshold(dist_transform, 0.4 * dist_transform.max(), 255, 0)
        self.processing_steps_images.append(dist_transform)
        self.processing_steps_images.append(sure_fg)


        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg, sure_fg)

        _, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1
        markers[unknown == 255] = 0
        cv2.watershed(self.img, markers)
        self.img[markers == -1] = [0, 0, 255]
        

        image_area = self.org_img.shape[0] * self.org_img.shape[1]
        max_allowed_area = image_area * 0.9  # 90% of the image area

        for label in np.unique(markers):
            if label <= 1: 
                continue
            mask = np.zeros(self.gray.shape, dtype = "uint8")
            mask[markers == label] = 255
            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                x, y, w, h = cv2.boundingRect(c)
                area = w * h

                # Skip the object if it covers more than 90% of the image
                if area > max_allowed_area:
                    continue

                cropped = self.org_img[y:y+h, x:x+w]
                cropped_resized = self.resize_with_padding(cropped)
                center_x = x + w // 2
                center_y = y + h // 2

                cropped_info = {
                    'cropped_image': cropped_resized,
                    'center_coordinates': (center_x, center_y),
                    'bounding_box': (x, y, w, h)  
                }
                self.cropped_objects_info.append(cropped_info)
                self.cropped_objects.append(cropped_resized)  
                self.all_cnts.append(cnts)
    
    def draw_rectangles(self):
        self.apply_watershed()
        display_img = self.org_img.copy()
        cv2.circle(display_img, (0, 0), radius=50, color=(0, 255, 0), thickness=-1)
        for cnts in self.all_cnts:
            for c in cnts:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(display_img, (x, y), (x + w, y + h), (0, 255, 0), 5)
                center_x = int(x + w / 2)
                center_y = int(y + h / 2)
                
                
                cv2.circle(display_img, (center_x, center_y), radius=20, color=(0, 255, 0), thickness=-1)
                cv2.putText(display_img, f"({center_x}, {center_y})", (center_x + 5, center_y + 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 0, 0), 12)

        return display_img
    
    def detect_objects(self):
        self.apply_watershed()
        object_centers = []
        for object_info in self.cropped_objects_info:
            object_centers.append(object_info['center_coordinates'])
        
        return object_centers
    
def GetObjects(frame):
    model = YOLO('./runs/100_epoch/train/weights/best.pt')
    results = model.predict(frame, max_det = 3)
    object_centers = []
    for box in results[0].boxes.xyxy:
        object_center = [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2]
        object_centers.append(np.array(object_center).reshape(-1, 2))
    
    all_objects = {}
    for i, label in enumerate(results[0]):
        all_objects[f'Obj{i}'] = {
            'label': label,
            'location': object_centers[i]
        }
    
    return all_objects

class PNPTestPath():
    def __init__(self, robot, object_points, green_box_location):
        self.robot = robot
        self.object_points = object_points
        self.green_box_location = green_box_location
        self.commands = None
        self.full_path = None
        self.CONVEYOR_SPEED = 50
        self.MOTOR_SPEED = 1000
        self.ANGLE_PER_STEP = 360/1600
    
    def create_elevated_spline(self, start_point, end_point, z_offset = 100):
        start_point_offset = start_point.copy()
        start_point_offset[2] += z_offset

        end_point_offset = end_point.copy()
        end_point_offset[2] += z_offset

        path_planner = PathPlanner(start_point, start_point_offset, end_point_offset, end_point, self.robot)
        
        return path_planner.generate_spline()
    
    def update_object_positions(self, objects, elapsed_time):
        # Update the y-coordinate based on conveyor speed and time
        updated_objects = objects.copy()
        updated_objects[:, 1] += self.CONVEYOR_SPEED * elapsed_time
        return updated_objects
    
    def calculate_steps_and_time(self, angle):
        steps = angle / self.ANGLE_PER_STEP
        time = steps / self.MOTOR_SPEED
        return steps, time
    
    def plan_path(self, time_since_framed=0):
        robot = self.robot
        commands = []
        objects = self.object_points.copy()
        home_pos = robot.calculate_fwd_kinematics(0, 0, 0)
        current_pos = home_pos
        full_path = []
        
        for i, obj in enumerate(objects):
            # Update object position based on total elapsed time so far
            objects[i, 0] += self.CONVEYOR_SPEED * time_since_framed

            if i == 0:
                path_to_obj = self.create_elevated_spline(current_pos, obj, z_offset = 100) # Go from home to first object
            else:
                path_to_obj = self.create_elevated_spline(current_pos, obj)
                
            full_path.extend(path_to_obj)
            
            for point in path_to_obj:
                motor_angles = self.robot.calculate_inv_kinematics(point[0], point[1], point[2])
                commands.append([*motor_angles, 1])
            
            
            # Plan path for the current object
            path_to_destination = self.create_elevated_spline(obj, self.green_box_location)
            full_path.extend(path_to_destination)
            last_picked_time = 0

            current_angle = 0
            last_angle = 0
            
            for point in path_to_destination:
                motor_angles = self.robot.calculate_inv_kinematics(point[0], point[1], point[2])
                current_angle = max(motor_angles)
                _, time_to_execute = self.calculate_steps_and_time(np.abs(current_angle - last_angle))
                last_picked_time += time_to_execute
                # print(f'Last picked time: {last_picked_time}')
                # Add commands including gripper state (assuming 1 for grip and 0 for release)
                commands.append([*motor_angles, 1])  # Gripping
                last_angle = current_angle
            
            # Releasing at the destination
            commands.append([*motor_angles, 0])  
            time_since_framed += last_picked_time

            # Update positions of remaining objects to account for conveyor movement during this pick
            for j in range(i + 1, len(objects)):
                objects[j, 1] += self.CONVEYOR_SPEED * last_picked_time
            
            current_pos = self.green_box_location
        # print(f'Total estimated time to complete path: {time_since_framed}')
        path_back_to_home = self.create_elevated_spline(self.green_box_location, home_pos)
        full_path.extend(path_back_to_home)
        
        for point in path_back_to_home:
            motor_angles = self.robot.calculate_inv_kinematics(point[0], point[1], point[2])
            commands.append([*motor_angles, 0])
        
        # Append All Paths
        self.full_path = full_path
        self.commands = commands
        # print(self.commands)
        return commands, full_path
        
    
    def plot_full_path(self):
        sns.set_style('whitegrid')
        x_coords = [p[0] for p in self.full_path]
        y_coords = [p[1] for p in self.full_path]
        z_coords = [p[2] for p in self.full_path]

        # Creating a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plotting the spline path
        ax.plot(x_coords, y_coords, z_coords, label='Spline Path', color = 'green')
        
        for i, point in enumerate(self.object_points):
            ax.text(point[0], point[1], point[2], f'O{i}', color='red')

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        ax.set_xlim([-600, 600])
        ax.set_ylim([-600, 600])
        ax.set_zlim([-600, 600])

        ax.legend()

        plt.show()
    
    def plot_angle_progression(self):
        plt.figure(figsize=(10, 6))

        # Plot each motor's progression over command sequence
        for i, (m1, m2, m3, g) in enumerate(self.commands):
            color_m1 = 'red' if g == 1 else 'blue'
            color_m2 = 'green' if g == 1 else 'blue'
            color_m3 = 'yellow' if g == 1 else 'blue'
            plt.scatter(i, m1, color = color_m1, label='M1' if i == 0 else "", marker='.', s=2)
            plt.scatter(i, m2, color=color_m2, label='M2' if i == 0 else "", marker='.', s = 2)
            plt.scatter(i, m3, color=color_m3, label='M3' if i == 0 else "", marker='.', s = 2)

        # Add labels, title, and legend
        plt.xlabel('Command Sequence Number')
        plt.ylabel('Motor Angle')
        plt.title('Motor Angles Over Command Sequence with Gripper State')
        plt.legend()

        # Show the plot
        plt.show()
  