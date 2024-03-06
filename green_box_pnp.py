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

# commands, full_path = plan_path(object_points, green_box_location)
# plot_full_path(full_path, object_points)
# plot_angle_progression(commands)
# Export full_path to csv, use floatformat = '%.3f' to reduce file size
# np.savetxt('full_path.csv', full_path, delimiter=',', fmt='%.2f')

# Connect to arduino and verify connection
# port = 'COM7'
# baudrate = 9600
# ard = None
# try:
#     ard = serial.Serial(port, baudrate = baudrate, timeout = 2)
#     time.sleep(2)  
#     if ard:
#         print(f"Connection to {port} established")
#         print('Sending commands...')
#         for m1,m2,m3,g in commands:
#             send_command(ser = ard, m1 = m1, m2 = m2, m3 = m3, gripper = g)
#             # print(f'Sent command: {m1}, {m2}, {m3}, {g}')
        
#         print('Commands sent successfully')

# except serial.SerialException as e:
#     print('Connection Error', f'Failed to connect: {str(e)}')


#Pack everything to a class

from essentials import *
        

test_path = PNPTestPath(robot, object_points, green_box_location)
commands, full_path = test_path.plan_path()
test_path.plot_full_path()