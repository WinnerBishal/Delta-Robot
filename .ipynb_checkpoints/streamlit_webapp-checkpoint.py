import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from delta_robot import DeltaRobot  # Assuming you have your DeltaRobot class in delta_robot.py

# Set up the sidebar for user inputs
st.sidebar.title("Configure the robot")
r = st.sidebar.number_input("r", value=1.0)
h = st.sidebar.number_input("h", value=1.0)
s = st.sidebar.number_input("s", value=1.0)
k = st.sidebar.number_input("k", value=1.0)
Zt = st.sidebar.number_input("Zt", value=0.1)

st.sidebar.title("Set joint angles")
J1 = st.sidebar.slider("J1", 0, 360, 0)
J2 = st.sidebar.slider("J2", 0, 360, 0)
J3 = st.sidebar.slider("J3", 0, 360, 0)

# Create the robot and calculate the TCP
robot = DeltaRobot(r, h, s, k, Zt)
TCP = robot.calculate_kinematics(J1, J2, J3)

# Display the TCP
st.title("TCP")
st.write(TCP)

# Plot the robot
fig, ax = plt.subplots()
# Add your plotting code here
# ax.plot(...)
st.pyplot(fig)
