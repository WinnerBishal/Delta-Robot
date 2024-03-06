import numpy as np 
import plotly.graph_objects as go
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time, random
from essentials import *
import serial

def adjust_for_conveyor_movement(object_point, time_to_reach):
    adjusted_point = object_point.copy()
    adjusted_point[1] += conveyor_speed_y * 1
    return adjusted_point

r = 171.5
h = 200
s = 50
k = 435
Zt = 0
robot = DeltaRobot(r, h, s, k, Zt)

# ardMega = serial.Serial('COM7', baudrate = 138400, timeout = 2)

conveyor_speed_y = 1  


class ConveyorRobot:
    def __init__(self, robot, z_offset=100):
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

conveyor_robot = ConveyorRobot(robot, z_offset = 100)

# object_points = [[0, 0, -500], [100, 0, -500], [200, 0, -500]]
box_points = [[-200, -230, -500], [200, -230, -500], [-300, -230, -500]]
green_box_location = [-200, -230, -500]

camera = cv2.VideoCapture(2)  

frame_processing_interval = 10  
last_processed_time = time.time()


while True:
    plt.close('all')
    ret, frame = camera.read()
    if not ret:
        break

    current_time = time.time()
    if current_time - last_processed_time >= frame_processing_interval:
        last_processed_time = current_time

        all_objects = GetObjects(frame)

        adjusted_robot_coordinates = []
        for object in all_objects:
            if object['label'] != 'green':
                continue
            actual_location = image2Dworld3D([object['location']])
            time_to_reach = 1
            print(actual_location)
            adjusted_robot_center = adjust_for_conveyor_movement(actual_location, time_to_reach)
            adjusted_robot_coordinates.append(adjusted_robot_center)

        if adjusted_robot_coordinates:
            random.shuffle(box_points)
            destination_boxes = box_points[:len(adjusted_robot_coordinates)]
            full_path = conveyor_robot.process_batch(adjusted_robot_coordinates, destination_boxes)
            # execute_path_on_robot(full_path)  
            
            x_coords = [p[0] for p in full_path]
            y_coords = [p[1] for p in full_path]
            z_coords = [p[2] for p in full_path]
            
            # Creating a 3D plot
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            print("here")
            ax.plot(x_coords, y_coords, z_coords, label='Spline Path')

            for point in box_points:
                ax.text(point[0], point[1], point[2], 'B', color='blue')
            for point in adjusted_robot_coordinates:
                ax.text(point[0], point[1], point[2], 'O', color='red')

            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            ax.set_xlim([-400, 400])
            ax.set_ylim([-400, 400])
            ax.set_zlim([-600, 600])

            ax.legend()

            plt.show()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()


def plot_full_path(full_path, adjusted_robot_coordinates):
    x_coords = [p[0] for p in full_path]
    y_coords = [p[1] for p in full_path]
    z_coords = [p[2] for p in full_path]

    # Creating a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plotting the spline path
    ax.plot(x_coords, y_coords, z_coords, label='Spline Path')

    # Labeling box points with 'B' and object points with 'O'
    for point in box_points:
        ax.text(point[0], point[1], point[2], 'B', color='blue')
    for point in adjusted_robot_coordinates:
        ax.text(point[0], point[1], point[2], 'O', color='red')

    # Setting labels and limits
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim([-400, 400])
    ax.set_ylim([-400, 400])
    ax.set_zlim([-600, 600])

    # Adding a legend (optional)
    ax.legend()

    # Showing the plot
    plt.show()
    

