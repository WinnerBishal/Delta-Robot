import streamlit as st
from streamlit import session_state as state
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from delta_robot import DeltaRobot  # Assuming you have your DeltaRobot class in delta_robot.py

# Create two columns
col1, col2 = st.columns([1, 1])

# Default robot parameters
r = 200
h = 300
s = 140
k = 500
Zt = 0
J1 = 0
J2 = 0
J3 = 0

st.sidebar.title("Configure Robot Parameters")

# Initialize session state
if 'configure_robot' not in state:
    state.configure_robot = False
if 'input_angles' not in state:
    state.input_angles = False

if st.sidebar.button("Configure Robot"):
    state.configure_robot = not state.configure_robot

if state.configure_robot:
    # Input fields for robot parameters
    r = st.sidebar.number_input("Base Radius (r)", value=200)
    h = st.sidebar.number_input("Bicep Length (h)", value=300)
    s = st.sidebar.number_input("End Effector Radius (s)", value=140)
    k = st.sidebar.number_input("Arm Length (k)", value=500)
    Zt = st.sidebar.number_input("Wrist Point to TCP Height (Zt)", value=0)

if st.sidebar.button("Input Angles"):
    state.input_angles = not state.input_angles

if state.input_angles:
    # Input fields for joint angles
    J1 = st.sidebar.number_input("J1", -90, 90, 0)
    J2 = st.sidebar.number_input("J2", -90, 90, 0)
    J3 = st.sidebar.number_input("J3", -90, 90, 0)

robot = DeltaRobot(r, h, s, k, Zt)
TCP = robot.calculate_kinematics(J1, J2, J3)

col2.title("Robot Calculations")
fwd_tab, inv_tab = col2.tabs(['Forward Kinematics ', 'Inverse Kinematics'])

# Display the TCP in the second column
fwd_tab.write(f"X: {TCP[0]:.2f}")
fwd_tab.write(f"Y: {TCP[1]:.2f}")
fwd_tab.write(f"Z: {TCP[2]:.2f}")

# Plot the robot in the first column
with col1:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Add your plotting code here
    st.pyplot(fig)
