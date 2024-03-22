from typing import Any
import matplotlib.pyplot as plt
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import time, random, serial
from essentials import *

CONVEYOR_SPEED = 50  
MOTOR_SPEED = 100  
ANGLE_PER_STEP = 1.8

r = 171.5
h = 200
s = 50
k = 435
Zt = 0
robot = DeltaRobot(r, h, s, k, Zt)
        
    
object_points = np.array([[0, 100, -500], [100, -100, -500], [200, 0, -500]])
green_box_location = np.array([100, -230, -500])
        

test_path = PNPTestPath(robot, object_points, green_box_location)
commands, full_path = test_path.plan_path()
test_path.plot_angle_progression()