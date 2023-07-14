import streamlit as st
from streamlit import session_state as state
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from delta_robot import DeltaRobot  # Assuming you have your DeltaRobot class in delta_robot.py

# Create two columns
col1, col2 = st.columns([1, 1])

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
Z_in = 0

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
    J1 = st.sidebar.number_input("J1", -90, 90, 0)
    J2 = st.sidebar.number_input("J2", -90, 90, 0)
    J3 = st.sidebar.number_input("J3", -90, 90, 0)


if st.sidebar.button("Input TCP"):
    state.input_TCP = not state.input_TCP

if state.input_TCP:
    X_in = st.sidebar.number_input("X", value = 0.0)
    Y_in = st.sidebar.number_input("Y", value = 0.0)
    Z_in = st.sidebar.number_input("Z", value = 0.0)

robot = DeltaRobot(r, h, s, k, Zt)
TCP = robot.calculate_kinematics(J1, J2, J3)
ANG = [0.0, 0.0, 0.0]

#Store results in dataframe
fwd_kine_df = pd.DataFrame({'Input Angles \n (J1, J2, J3)': [(J1, J2, J3)], 'Calculated TCP \n (X, Y, Z)': [(TCP[0], TCP[1], TCP[2])]})
inv_kine_df = pd.DataFrame({'Input TCP \n (X, Y, Z)': [(X_in, Y_in, Z_in)], 'Calculated Angles \n (J1, J2, J3)': [(ANG[0],ANG[1],ANG[2])]})

col2.header("Robot Calculation Results")
fwd_tab, inv_tab = col2.tabs(['Forward Kinematics Results', 'Inverse Kinematics Results'])

# Display the TCP results in forward knematics tab
fwd_tab.dataframe(fwd_kine_df, hide_index = True)


# Display angle results in inverse Kinematics tab
inv_tab.dataframe(inv_kine_df, hide_index = True)

# Plot the robot in the first column
col1.header("Visualization of Delta Robot Kinematics")
with col1:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Add your plotting code here
    st.pyplot(fig)
