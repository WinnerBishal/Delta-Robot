import streamlit as st
from streamlit import session_state as state
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from delta_robot import DeltaRobot, plot_delta_robot  # Assuming DeltaRobot class is in delta_robot.py

# Create two columns
col1, col2 = st.tabs(['Visualize Delta Robot Schematic for Input Angle', 'Calculate Forward & Reverse Kinematics'])

# Default robot parameters, initialize variables
r = 200
h = 300
s = 140
k = 500
Zt = 0
J1 = 0
J2 = 0
J3 = 0
X_in = 0
Y_in = 0
Z_in = -347

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
    h = st.sidebar.number_input("Bicep Length (h)", value=300)
    s = st.sidebar.number_input("End Effector Radius (s)", value=140)
    k = st.sidebar.number_input("Arm Length (k)", value=500)
    Zt = st.sidebar.number_input("Wrist Point to TCP Height (Zt)", value=0)

st.sidebar.title("Input for Calculations")
if st.sidebar.button("Input Angles"):
    state.input_angles = not state.input_angles

if state.input_angles:
    # Input fields for joint angles
    J1 = st.sidebar.number_input("J1", -90.00, 90.00, value = 0.00)
    J2 = st.sidebar.number_input("J2", -90.00, 90.00, value = 0.00)
    J3 = st.sidebar.number_input("J3", -90.00, 90.00, value = 0.00)


if st.sidebar.button("Input TCP"):
    state.input_TCP = not state.input_TCP

if state.input_TCP:
    X_in = st.sidebar.number_input("X", -400.00, 400.00, value = 0.00)
    Y_in = st.sidebar.number_input("Y", -400.00, 400.00, value = 0.00)
    Z_in = st.sidebar.number_input("Z", -600.00, 200.00, value = -347.00)

robot = DeltaRobot(r, h, s, k, Zt)
TCP = robot.calculate_fwd_kinematics(J1, J2, J3)
ANG = robot.calculate_inv_kinematics(X_in, Y_in, Z_in)

#Store results in dataframe
fwd_kine_df = pd.DataFrame({'Input Angles \n (J1, J2, J3)': [(J1, J2, J3)], 'Calculated TCP \n (X, Y, Z)': [(TCP[0], TCP[1], TCP[2])]})
inv_kine_df = pd.DataFrame({'Input TCP \n (X, Y, Z)': [(X_in, Y_in, Z_in)], 'Calculated Angles \n (J1, J2, J3)': [(ANG[0],ANG[1],ANG[2])]})


fwd_tab, inv_tab = col2.tabs(['Forward Kinematics Results', 'Inverse Kinematics Results'])

# Display the TCP results in forward knematics tab
fwd_tab.dataframe(fwd_kine_df, hide_index = True)


# Display angle results in inverse Kinematics tab
inv_tab.dataframe(inv_kine_df, hide_index = True)

# Plot the robot in the first column
with col1:
    fig = plot_delta_robot(robot.B1, robot.B2, robot.B3, robot.WP, robot.r, robot.s)
    st.plotly_chart(fig, use_container_width= True)

