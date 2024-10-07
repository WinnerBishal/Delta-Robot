# Industrial Delta Robot

## Overview

This project focuses on the **design, fabrication, and testing** of a **camera-equipped Delta robot** for advanced grading in conveyor systems. The Delta robot automates the grading process, improving speed and precision, especially for applications in the food processing industry.

### Key Features:
- Object detection and classification using **YOLO v8** : https://github.com/WinnerBishal/Delta-Robot/tree/master/computer_vision
- Delta Robot Forward and Inverse Kinematics in Python : https://github.com/WinnerBishal/Delta-Robot/blob/master/src/essentials.py
- Webapp for interactive visualization of Kinematics and Spline Path Planning : https://bishaladhikari.info.np/delta_kinematics_calculator/
- Arduino code for simultaneous operation of three stepper motors based on computer based commands : https://github.com/WinnerBishal/Delta-Robot/tree/master/arduino
- Python GUI for interacting with actual robot : https://github.com/WinnerBishal/Delta-Robot/tree/master/delta_gui
- All Solidworks Design and Assembly Files : https://github.com/WinnerBishal/Delta-Robot/tree/master/solidworks_design_files
- Find ROS2 code in another repository : https://github.com/WinnerBishal/ROS2_delta_robot



https://github.com/user-attachments/assets/9670b60e-34f7-4394-b9da-4f849989f6b0



## Project Objectives

- Design and fabricate a prototype of a **Delta robot** for vision-based grading.
- Implement **autonomous pick-and-place** using a camera and vacuum gripper.
- Integrate a **ROS2-based control system** for smooth operations.
- Test and optimize the performance for real-world grading tasks.

---

## System Design

### Mechanical Design
- Developed using **SolidWorks** and **3D printed** components.
- Key parts include the **base frame**, **bicep rod**, and **end effector**.

![image](https://github.com/user-attachments/assets/dffad3bb-8ded-44e9-ad8c-4d978c0e4584)


### Electronics System
- **Arduino Mega** for controlling motors and sensors.
- **NEMA 23 stepper motors** and **TB6600 drivers** for precise movements.
- Includes **gyroscopic sensors**, **relay modules**, and a **30A power supply**.

![image](https://github.com/user-attachments/assets/1f6c1b0d-a91a-4d25-8d1e-2dc4bcb629e4)


### Vision System
- Uses **YOLO v8** for object detection.
- The camera system identifies and grades objects based on visual parameters.
- Accurate positioning using **coordinate transformations**.

![image](https://github.com/user-attachments/assets/2acf39f6-8774-4c08-9f28-0ad6f014be38)
![image](https://github.com/user-attachments/assets/c2b58cc2-8ba4-48b4-ba5a-a24972e11e3b)


### Operating Software
- **Python** for image processing and GUI.
- **Arduino** for serial communication and actuator control.
- **ROS2** for autonomous system operations and smooth trajectory control.

![image](https://github.com/user-attachments/assets/3fcf0a76-8b69-4d72-bc47-9ae8f18456da)


---

## Results and Discussion

- Achieved **900 tomatoes per hour** in static conditions.
- Reliable performance in **pick-and-place** operations.
- Improved **speed and accuracy**, making it suitable for industrial applications.

![image](https://github.com/user-attachments/assets/3d78b62d-d4e1-4479-8cef-283272d05044)


---

## Challenges and Future Development

### Challenges:
- **Pneumatic control** could be improved for faster operations.
- **Mechanical imperfections** leading to minor errors in positioning.

### Future Development:
- Integrate a **dynamic conveyor system**.
- Enhance programming flexibility for **better control**.
- Improve **structural stability** and add more **safety features**.

---

## Conclusion

The project successfully demonstrated the integration of **mechanical design, electronics, and vision systems** to automate grading processes. The **Delta robot** proved effective in improving the efficiency of grading systems in the food processing industry.

---

## How to Use

1. Clone the repository:
   ```bash
   git clone https://github.com/WinnerBishal/Delta-Robot.git
