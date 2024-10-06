import streamlit as st
from streamlit import session_state as state
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from essentials import *  # Assuming all the necessary functions are in essentials.py
# Create two columns
col1, col2, col3 = st.tabs(['Visualize Delta Robot Schematic for Input Angle', 'Calculate Forward & Reverse Kinematics', 'Path Planning'])

# Default robot parameters, initialize variables
r = 171.5
h = 200
s = 50
k = 435
Zt = 0
J1 = 0
J2 = 0
J3 = 0
X_in = 0
Y_in = 0
Z_in = -325

# Widget for configuring robot parameters
st.sidebar.title("Configure Robot Parameters")

# Initialize session state
if 'configure_robot' not in state:
    state.configure_robot = False
if 'input_angles' not in state:
    state.input_angles = False
if 'input_TCP' not in state:
    state.input_TCP = False

if st.sidebar.button("Configure Robot"):
    state.configure_robot = not state.configure_robot

if state.configure_robot:
    # Input fields for robot parameters
    r = st.sidebar.number_input("Base Radius (r)", value=200)
    h = st.sidebar.number_input("Bicep Length (h)", value=200)
    s = st.sidebar.number_input("End Effector Radius (s)", value=50)
    k = st.sidebar.number_input("Arm Length (k)", value=400)
    Zt = st.sidebar.number_input("Wrist Point to TCP Height (Zt)", value=0)

st.sidebar.title("Input for Calculations")
if st.sidebar.button("Input Angles"):
    state.input_angles = not state.input_angles

if state.input_angles:
    # Input fields for joint angles
    J1 = st.sidebar.number_input("J1", -90, 90, value = 0)
    J2 = st.sidebar.number_input("J2", -90, 90, value = 0)
    J3 = st.sidebar.number_input("J3", -90, 90, value = 0)


if st.sidebar.button("Input TCP"):
    state.input_TCP = not state.input_TCP

if state.input_TCP:
    X_in = st.sidebar.number_input("X", -400, 400, value = 0)
    Y_in = st.sidebar.number_input("Y", -400, 400, value = 0)
    Z_in = st.sidebar.number_input("Z", -600, 200, value = -200)

robot = DeltaRobot(r, h, s, k, Zt)
TCP = robot.calculate_fwd_kinematics(J1, J2, J3)
ANG = robot.calculate_inv_kinematics(X_in, Y_in, Z_in)

#Store results in dataframe
fwd_kine_df = pd.DataFrame({'Input Angles \n (J1, J2, J3)': [(J1, J2, J3)], 'Calculated TCP \n (X, Y, Z)': [(TCP[0], TCP[1], TCP[2])]})
inv_kine_df = pd.DataFrame({'Input TCP \n (X, Y, Z)': [(X_in, Y_in, Z_in)], 'Calculated Angles \n (J1, J2, J3)': [(ANG[0],ANG[1],ANG[2])]})


col2.write("## Forward Kinematics")
# Display TCP results in forward Kinematics tab
col2.dataframe(fwd_kine_df, hide_index = True)
col2.divider()
col2.write("## Inverse Kinematics")
# Display angle results in inverse Kinematics tab
col2.dataframe(inv_kine_df, hide_index = True)

# Plot the robot in the first column
with col1:
    robot_fig = plot_delta_robot(robot.B1, robot.B2, robot.B3, robot.WP, robot.r, robot.s)
    st.plotly_chart(robot_fig, use_container_width= True)

# Number Input to get coordinates of points P0, P1, P2, P3
col3.write("### Input Coordinates of Points P0, P1, P2, P3")
sp_coord_tab = col3.expander("Click to input Co-ordinates", expanded = False)

P0_x = sp_coord_tab.number_input("P0_x", -400.00, 400.00, value = 0.00)
P0_y = sp_coord_tab.number_input("P0_y", -400.00, 400.00, value = 0.00)
P0_z = sp_coord_tab.number_input("P0_z", -600.00, 200.00, value = -347.00)

P1_x = sp_coord_tab.number_input("P1_x", -400.00, 400.00, value = 0.00)
P1_y = sp_coord_tab.number_input("P1_y", -400.00, 400.00, value = 0.00)
P1_z = sp_coord_tab.number_input("P1_z", -600.00, 200.00, value = -247.00)

P2_x = sp_coord_tab.number_input("P2_x", -400.00, 400.00, value = 0.00)
P2_y = sp_coord_tab.number_input("P2_y", -400.00, 400.00, value = 350.00)
P2_z = sp_coord_tab.number_input("P2_z", -600.00, 200.00, value = -247.00)

P3_x = sp_coord_tab.number_input("P3_x", -400.00, 400.00, value = 0.00)
P3_y = sp_coord_tab.number_input("P3_y", -400.00, 400.00, value = 350.00)
P3_z = sp_coord_tab.number_input("P3_z", -600.00, 200.00, value = -347.00)


# Initialize  start, end and control points
P0 = np.array([P0_x, P0_y, P0_z])
P1 = np.array([P1_x, P1_y, P1_z])
P2 = np.array([P2_x, P2_y, P2_z])
P3 = np.array([P3_x, P3_y, P3_z])

planner = PathPlanner(P0, P1, P2, P3, robot)
planner.perform_spline_planning()

spline_tab, line_tab = col3.tabs(['Spline Path Planning', 'Line Path Planning'])

with spline_tab:
    fig = planner.plot_spline(robot_fig)
    st.plotly_chart(fig, use_container_width= False)

spline_tab.divider()
with spline_tab:
    fig = planner.plot_spline_joint_space()
    st.plotly_chart(fig, use_container_width= False)

planner.perform_line_planning()

with line_tab:
    fig = planner.plot_line(robot_fig)
    st.plotly_chart(fig, use_container_width= False)

line_tab.divider()

with line_tab:
    fig = planner.plot_line_joint_space()
    st.plotly_chart(fig, use_container_width= False)