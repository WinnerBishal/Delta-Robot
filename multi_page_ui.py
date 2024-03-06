import sys, time, random
import cv2

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QSlider, QPushButton, QVBoxLayout, QMessageBox, QMainWindow, QStackedWidget, QHBoxLayout, QRadioButton, QButtonGroup, QGridLayout, QSpinBox
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QTimer, Qt
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtGui import QImage, QPixmap

import serial
from plotly.offline import plot

from essentials import *

# Essential Global variables
mega_baudrate = 38400
ardMega = None

# Delta Robot Parameters
r = 171.5
h = 200
s = 50
k = 435
Zt = 0
robot = DeltaRobot(r, h, s, k, Zt)


class RobotConnection(QWidget):
    
    connection_successful = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # Layout
        vbox = QVBoxLayout()

        # Arduino Port Label
        self.label = QLabel('Arduino Port', self)

        # Arduino Port Entry
        self.port_entry = QLineEdit(self)

        # Connect Button
        self.connect_button = QPushButton('CONNECT', self)
        self.connect_button.clicked.connect(self.on_connect)
        self.connect_button.setObjectName('primary')

        # Error Message Display Box
        self.error_display = QMessageBox(self)

        # Adding widgets to the layout
        vbox.addWidget(self.label)
        vbox.addWidget(self.port_entry)
        vbox.addWidget(self.connect_button)

        self.setLayout(vbox)
        self.setWindowTitle('Robot Connection')

    @pyqtSlot()
    def on_connect(self):
        global ardMega
        # Get the port from the entry box
        port = self.port_entry.text()
        try:
            ardMega = serial.Serial(port, baudrate = mega_baudrate, timeout = 2)
            time.sleep(2)  
            if ardMega:
                print(f"Connection to {port} established")
            self.connection_successful.emit()
            
        except serial.SerialException as e:
            self.error_display.critical(self, 'Connection Error', f'Failed to connect: {str(e)}')
        
        
            
class CalibrationPage(QWidget):
    calibration_complete = pyqtSignal()
    skip_calibration = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.initUI()
        self.currentCalibrationIndex = 0
        self.calibrationTimer = QTimer(self)
        self.calibrationTimer.timeout.connect(self.handle_calibration_success)
        self.calibrationTimer.setSingleShot(True)

    def initUI(self):
        vbox = QVBoxLayout()
        self.calibrationAxes = ['M1', 'M2', 'M3']
        self.instruction_label = QLabel(f'Place the calibration tool on the motor {self.calibrationAxes[0]} and click the button below.', self)
        self.calibrate_button = QPushButton(f'Calibrate {self.calibrationAxes[0]} motor', self)
        self.calibrate_button.clicked.connect(self.on_calibrate)
        self.calibrate_button.setObjectName('primary')
        self.goBack_button = QPushButton('Go Back', self)
        self.goBack_button.clicked.connect(self.on_go_back)
        self.goBack_button.setObjectName('secondary')
        self.goBack_button.setEnabled(False)
        self.skip_button = QPushButton('Skip Calibration', self)
        self.skip_button.setObjectName('secondary')
        self.skip_button.clicked.connect(self.skip_calibration.emit)
        vbox.addWidget(self.instruction_label)
        vbox.addWidget(self.calibrate_button)
        vbox.addWidget(self.goBack_button)
        vbox.addWidget(self.skip_button)
        
        
        self.setLayout(vbox)
        
    
    def on_calibrate(self):
        print("Calibrate button clicked")  # Debug message
        try:
            if self.currentCalibrationIndex < len(self.calibrationAxes):
                axis = self.currentCalibrationIndex + 1
                send_command(ardMega, calibrate_motor=axis)
                self.calibrationTimer.start(30000)  # Start 20 seconds timer
                print("Timer started")  # Debug message
        except Exception as e:
            print(f"An error occurred: {e}")
            QMessageBox.critical(self, "Error", str(e))
            
        self.goBack_button.setEnabled(True)

    def on_go_back(self):
        if self.currentCalibrationIndex > 0:
            self.currentCalibrationIndex -= 1
            previous_axis = self.calibrationAxes[self.currentCalibrationIndex]
            self.instruction_label.setText(f'Place the calibration tool on the motor {previous_axis} and click the button below.')
            self.calibrate_button.setText(f'Calibrate {previous_axis} motor')
            self.goBack_button.setEnabled(False)
            
            if self.currentCalibrationIndex == 0:
                self.goBack_button.setEnabled(False)
                
    def handle_calibration_success(self):
        print("Calibration success handler called")  # Debug message
        QMessageBox.information(self, "Calibration", f"Calibration complete for motor {self.calibrationAxes[self.currentCalibrationIndex]}")
        self.currentCalibrationIndex += 1
        
        if self.currentCalibrationIndex < len(self.calibrationAxes):
            next_axis = self.calibrationAxes[self.currentCalibrationIndex]
            self.instruction_label.setText(f'Place the calibration tool on the motor {next_axis} and click the button below.')
            self.calibrate_button.setText(f'Calibrate {next_axis} motor')
            
        elif self.currentCalibrationIndex == len(self.calibrationAxes) - 1:
            self.goBack_button.setEnabled(False)
        else:
            self.calibration_complete.emit()


class HomePage(QWidget):
    manual_op_req = pyqtSignal()
    tomato_sorting_req = pyqtSignal()
    back_to_calibration = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        vbox = QVBoxLayout()

        self.manual_operation_button = QPushButton('Manual Operation', self)
        self.manual_operation_button.setObjectName('primary')
        self.tomato_sorting_button = QPushButton('Tomato Sorting', self)
        self.tomato_sorting_button.setObjectName('primary')
        self.back_to_calibration_button = QPushButton('Back To Calibration', self)
        self.back_to_calibration_button.setObjectName('secondary')

        self.manual_operation_button.clicked.connect(self.on_manual_operation)
        self.tomato_sorting_button.clicked.connect(self.on_tomato_sorting)
        self.back_to_calibration_button.clicked.connect(self.back_to_calibration.emit)
        
        vbox.addWidget(self.manual_operation_button)
        vbox.addWidget(self.tomato_sorting_button)
        vbox.addWidget(self.back_to_calibration_button)
        
        self.setLayout(vbox)

    def on_manual_operation(self):
        self.manual_op_req.emit()
    
    def on_tomato_sorting(self):
        self.tomato_sorting_req.emit()
        

class ManualOperationPage(QWidget):

    go_back = pyqtSignal()

    m1_min, m2_min, m3_min = [-45, -45, -45]
    m1_max, m2_max, m3_max = [45, 45, 45]
    
    def __init__(self):
        super().__init__()
        self.motor_speed = 100  
        self.gripper_state = 'OPEN'
        self.mode = 'Move To Position'
        self.positioning_mode = 'Exact'
        
        self.x_val = 0
        self.y_val = 0
        self.z_val = -300
        
        self.m1 = 0
        self.m2 = 0
        self.m3 = 0
        
        self.increment = 10
        
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Motor Speed Control
        motor_speed_layout = QHBoxLayout()
        self.motor_speed_label = QLabel(f'Motor Speed: {self.motor_speed}', self)
        self.speed_plus_button = QPushButton('+', self)
        self.speed_plus_button.clicked.connect(self.increase_motor_speed)
        self.speed_minus_button = QPushButton('-', self)
        self.speed_minus_button.clicked.connect(self.decrease_motor_speed)
        
        motor_speed_layout.addWidget(self.motor_speed_label)
        motor_speed_layout.addWidget(self.speed_plus_button)
        motor_speed_layout.addWidget(self.speed_minus_button)

        layout.addLayout(motor_speed_layout)

         # Gripper Control
        self.gripper_label = QLabel('Gripper:', self)
        
        self.gripper_button = QPushButton(self.gripper_state, self)
        self.gripper_button.setObjectName('gripperOpen')
        self.gripper_button.clicked.connect(self.toggle_gripper)
        
        gripper_layout = QHBoxLayout()
        gripper_layout.addWidget(self.gripper_label)
        gripper_layout.addWidget(self.gripper_button)
        
        layout.addLayout(gripper_layout)
        
        # Mode Selection
        self.mode_group = QButtonGroup(self)
        
        self.move_to_position_radio = QRadioButton('Move To Position', self)
        self.move_joints_radio = QRadioButton('Move Joints', self)
        self.mode_group.addButton(self.move_to_position_radio)
        self.mode_group.addButton(self.move_joints_radio)

        self.move_to_position_radio.setChecked(True)
        self.move_to_position_radio.toggled.connect(self.toggle_mode)
        self.move_joints_radio.toggled.connect(self.toggle_mode)
        
        mode_selection_layout = QHBoxLayout()
        mode_selection_layout.addWidget(self.move_to_position_radio)
        mode_selection_layout.addWidget(self.move_joints_radio)
        layout.addLayout(mode_selection_layout)
        
        # "Move To Position Mode' Toggles
        self.position_mode_group = QButtonGroup(self)
        
        self.exact_position_radio = QRadioButton('Exact', self)
        self.increment_position_radio = QRadioButton('Increment', self)
        self.position_mode_group.addButton(self.exact_position_radio)
        self.position_mode_group.addButton(self.increment_position_radio)
        
        self.exact_position_radio.setChecked(True)
        
        self.exact_position_radio.toggled.connect(self.toggle_position_mode)
        self.increment_position_radio.toggled.connect(self.toggle_position_mode)
        
        position_mode_layout = QHBoxLayout()
        position_mode_layout.addWidget(self.exact_position_radio)
        position_mode_layout.addWidget(self.increment_position_radio)
        layout.addLayout(position_mode_layout)
        
        # Exact and Increment input Controls for Positioning
        self.position_controls_layout = QGridLayout()
        
        self.x_input = QLineEdit(self)
        self.y_input = QLineEdit(self)
        self.z_input = QLineEdit(self)
        
        self.m1_input = QLineEdit(self)
        self.m2_input = QLineEdit(self)
        self.m3_input = QLineEdit(self)
        
        self.x_plus_button = QPushButton('+X', self)
        self.x_plus_button.setObjectName('plusButton')
        self.x_minus_button = QPushButton('-X', self)
        self.x_minus_button.setObjectName('minusButton')
        self.y_plus_button = QPushButton('+Y', self)
        self.y_plus_button.setObjectName('plusButton')
        self.y_minus_button = QPushButton('-Y', self)
        self.y_minus_button.setObjectName('minusButton')
        self.z_plus_button = QPushButton('+Z', self)
        self.z_plus_button.setObjectName('plusButton')
        self.z_minus_button = QPushButton('-Z', self)
        self.z_minus_button.setObjectName('minusButton')
        
        self.m1_plus_button = QPushButton('+M1', self)
        self.m1_plus_button.setObjectName('plusButton')
        self.m1_minus_button = QPushButton('-M1', self)
        self.m1_minus_button.setObjectName('minusButton')
        self.m2_plus_button = QPushButton('+M2', self)
        self.m2_plus_button.setObjectName('plusButton')
        self.m2_minus_button = QPushButton('-M2', self)
        self.m2_minus_button.setObjectName('minusButton')
        self.m3_plus_button = QPushButton('+M3', self)
        self.m3_plus_button.setObjectName('plusButton')
        self.m3_minus_button = QPushButton('-M3', self)
        self.m3_minus_button.setObjectName('minusButton')
        
        self.m1_zero_button = QPushButton('Zero M1', self)
        self.m2_zero_button = QPushButton('Zero M2', self)
        self.m3_zero_button = QPushButton('Zero M3', self)
        
        self.position_controls_layout.addWidget(QLabel('X:'), 0, 0)
        self.position_controls_layout.addWidget(self.x_input, 0, 1)
        self.position_controls_layout.addWidget(self.x_plus_button, 0, 2)
        self.position_controls_layout.addWidget(self.x_minus_button, 0, 3)
        
        self.position_controls_layout.addWidget(QLabel('Y:'), 1, 0)
        self.position_controls_layout.addWidget(self.y_input, 1, 1)
        self.position_controls_layout.addWidget(self.y_plus_button, 1, 2)
        self.position_controls_layout.addWidget(self.y_minus_button, 1, 3)
        
        self.position_controls_layout.addWidget(QLabel('Z:'), 2, 0)
        self.position_controls_layout.addWidget(self.z_input, 2, 1)
        self.position_controls_layout.addWidget(self.z_plus_button, 2, 2)
        self.position_controls_layout.addWidget(self.z_minus_button, 2, 3)
        
        self.position_controls_layout.addWidget(QLabel('M1:'), 0, 4)
        self.position_controls_layout.addWidget(self.m1_input, 0, 5)
        self.position_controls_layout.addWidget(self.m1_plus_button, 0, 6)
        self.position_controls_layout.addWidget(self.m1_minus_button, 0, 7)
        self.position_controls_layout.addWidget(self.m1_zero_button, 0, 8)
        
        self.position_controls_layout.addWidget(QLabel('M2:'), 1, 4)
        self.position_controls_layout.addWidget(self.m2_input, 1, 5)
        self.position_controls_layout.addWidget(self.m2_plus_button, 1,6)
        self.position_controls_layout.addWidget(self.m2_minus_button, 1, 7)
        self.position_controls_layout.addWidget(self.m2_zero_button, 1, 8)
        
        self.position_controls_layout.addWidget(QLabel('M3:'), 2, 4)
        self.position_controls_layout.addWidget(self.m3_input, 2, 5)
        self.position_controls_layout.addWidget(self.m3_plus_button, 2, 6)
        self.position_controls_layout.addWidget(self.m3_minus_button, 2, 7)
        self.position_controls_layout.addWidget(self.m3_zero_button, 2, 8)
        
        self.val_input = QLineEdit(self)
        self.val_input.setText(str(self.increment))
        self.position_controls_layout.addWidget(QLabel('Increment Val:'), 4, 2)
        self.position_controls_layout.addWidget(self.val_input, 4, 3)
        
        self.move_to_pos_button = QPushButton('MOVE to POS', self)
        self.move_to_pos_button.setObjectName('primary')
        self.move_to_pos_button.clicked.connect(self.move_to_pos)
        self.position_controls_layout.addWidget(self.move_to_pos_button, 4, 1)
        
        self.move_to_angles_button = QPushButton('MOVE to ANGLE', self)
        self.move_to_angles_button.clicked.connect(self.move_to_angles)
        self.move_to_angles_button.setObjectName('primary')
        self.position_controls_layout.addWidget(self.move_to_angles_button, 4, 5)
        
        layout.addLayout(self.position_controls_layout)
        
        
        self.x_plus_button.clicked.connect(lambda: self.adjust_position('x', int(self.val_input.text())))
        self.x_minus_button.clicked.connect(lambda: self.adjust_position('x', -int(self.val_input.text())))
        
        self.y_plus_button.clicked.connect(lambda: self.adjust_position('y', int(self.val_input.text())))
        self.y_minus_button.clicked.connect(lambda: self.adjust_position('y', -int(self.val_input.text())))
        
        self.z_plus_button.clicked.connect(lambda: self.adjust_position('z', int(self.val_input.text())))
        self.z_minus_button.clicked.connect(lambda: self.adjust_position('z', -int(self.val_input.text())))
        
        self.m1_plus_button.clicked.connect(lambda: self.adjust_position('m1', int(self.val_input.text())))
        self.m1_minus_button.clicked.connect(lambda: self.adjust_position('m1', -int(self.val_input.text())))
        
        self.m2_plus_button.clicked.connect(lambda: self.adjust_position('m2', int(self.val_input.text())))
        self.m2_minus_button.clicked.connect(lambda: self.adjust_position('m2', -int(self.val_input.text())))
        
        self.m3_plus_button.clicked.connect(lambda: self.adjust_position('m3', int(self.val_input.text())))
        self.m3_minus_button.clicked.connect(lambda: self.adjust_position('m3', -int(self.val_input.text())))
        
        self.m1_zero_button.clicked.connect(lambda: self.zero_m1_motor())
        self.m2_zero_button.clicked.connect(lambda: self.zero_m2_motor())
        self.m3_zero_button.clicked.connect(lambda: self.zero_m3_motor())
        
        # Back to Homepage Button
        self.back_button = QPushButton('Back To Homepage', self)
        self.back_button.clicked.connect(self.on_back_to_homepage)
        
        self.auto_path_button = QPushButton('Auto Path', self)
        self.auto_path_button.clicked.connect(self.on_auto_path)
        
        self.pnp_test_path = QPushButton('PNP Test Path', self)
        self.pnp_test_path.clicked.connect(self.on_pnp_test_path)
        
        self.visualize_label = QLabel('Visualization', self)
        self.visualize_label.setAlignment(Qt.AlignCenter)
        
        robot.calculate_fwd_kinematics(self.m1, self.m2, self.m3)   
        fig = plot_delta_robot(robot.B1, robot.B2, robot.B3, robot.WP, robot.r, robot.s)
        plot_div = plot(fig , output_type = 'div', include_plotlyjs = 'cdn')
        self.web_engine_view = QWebEngineView()
        self.web_engine_view.setHtml(plot_div)
        
        layout.addWidget(self.auto_path_button)
        layout.addWidget(self.pnp_test_path)
        layout.addWidget(self.back_button)
        layout.addWidget(self.visualize_label)
        layout.addWidget(self.web_engine_view)
        
        self.setLayout(layout)

    def increase_motor_speed(self):
        self.motor_speed += 1  
        self.motor_speed_label.setText(f'Motor Speed: {self.motor_speed}')

    def decrease_motor_speed(self):
        if self.motor_speed > 0:
            self.motor_speed -= 1  
            self.motor_speed_label.setText(f'Motor Speed: {self.motor_speed}')
    
    def toggle_gripper(self):
        if self.gripper_state == 'OPEN':
            self.gripper_state = 'CLOSE'
            self.gripper_button.setObjectName('gripperClose')
            send_command(ardMega, gripper = 1)
        else:
            self.gripper_state = 'OPEN'
            send_command(ardMega, gripper = 0)
            self.gripper_button.setObjectName('gripperOpen')
            
        self.gripper_button.setText(self.gripper_state)
    
    def toggle_mode(self, checked):
        if checked:
            if self.move_to_position_radio.isChecked():
                self.mode = 'Move To Position'
            elif self.move_joints_radio.isChecked():
                self.mode = 'Move Joints'
            print(f"Mode switched to: {self.mode}")
    
    def toggle_position_mode(self, checked):
        if checked:
            if self.exact_position_radio.isChecked():
                self.positioning_mode = 'Exact'
            elif self.increment_position_radio.isChecked():
                self.positioning_mode = 'Increment'
            print(f"Positioning mode switched to: {self.positioning_mode}")
    
    def adjust_position(self, axis, adjustment):
        
        if self.mode == 'Move To Position' and self.positioning_mode == 'Increment':
            if axis =='x':
                self.x_val += adjustment
                self.x_input.setText(str(self.x_val))
                
                self.m1, self.m2, self.m3 = robot.calculate_inv_kinematics(self.x_val, self.y_val, self.z_val)
                send_command(ardMega, m1 = self.m1, m2 = self.m2, m3 = self.m3)
                
                self.m1_input.setText(str(self.m1))
                self.m2_input.setText(str(self.m2))
                self.m3_input.setText(str(self.m3))
                
                self.update_visualization(self.m1, self.m2, self.m3)
                
            elif axis =='y':
                self.y_val += adjustment
                self.y_input.setText(str(self.y_val))
                
                self.m1, self.m2, self.m3 = robot.calculate_inv_kinematics(self.x_val, self.y_val, self.z_val)
                send_command(ardMega, m1= self.m1, m2 = self.m2, m3 = self.m3)
                
                self.m1_input.setText(str(self.m1))
                self.m2_input.setText(str(self.m2))
                self.m3_input.setText(str(self.m3))
                
                self.update_visualization(self.m1, self.m2, self.m3)
                
            elif axis =='z':
                self.z_val += adjustment
                self.z_input.setText(str(self.z_val))
                
                self.m1, self.m2, self.m3 = robot.calculate_inv_kinematics(self.x_val, self.y_val, self.z_val)
                send_command(ardMega, m1 = self.m1, m2 = self.m2, m3 = self.m3)
                
                self.m1_input.setText(str(self.m1))
                self.m2_input.setText(str(self.m2))
                self.m3_input.setText(str(self.m3))
                
                self.update_visualization(self.m1, self.m2, self.m3)
            
        if self.mode == 'Move Joints' and self.positioning_mode == 'Increment':
            if axis =='m1':
                self.m1 += adjustment
                self.m1_input.setText(str(self.m1))
                
                send_command(ardMega, m1=self.m1)
                
                self.x_val, self.y_val, self.z_val = robot.calculate_fwd_kinematics(self.m1, self.m2, self.m3)
                self.x_input.setText(str(self.x_val))
                self.y_input.setText(str(self.y_val))
                self.z_input.setText(str(self.z_val))
                
                self.update_visualization(self.m1, self.m2, self.m3)
                
            elif axis =='m2':
                self.m2 += adjustment
                self.m2_input.setText(str(self.m2))
                
                send_command(ardMega, m2=self.m2)
                
                self.x_val, self.y_val, self.z_val = robot.calculate_fwd_kinematics(self.m1, self.m2, self.m3)
                self.x_input.setText(str(self.x_val))
                self.y_input.setText(str(self.y_val))
                self.z_input.setText(str(self.z_val))
                
                self.update_visualization(self.m1, self.m2, self.m3)
                
            elif axis =='m3':
                self.m3 += adjustment
                self.m3_input.setText(str(self.m3))
                
                send_command(ardMega, m3=self.m3)
                
                self.x_val, self.y_val, self.z_val = robot.calculate_fwd_kinematics(self.m1, self.m2, self.m3)
                self.x_input.setText(str(self.x_val))
                self.y_input.setText(str(self.y_val))
                self.z_input.setText(str(self.z_val))
                
                self.update_visualization(self.m1, self.m2, self.m3)
    
    def on_auto_path(self):
        P0 = [0, 0, -300]
        P1 = [0, 0, -510]
        P2 = [0, 0, -300]
        P3 = [0, -225, -300]
        
        M0 = robot.calculate_inv_kinematics(P0[0], P0[1], P0[2])
        M1 = robot.calculate_inv_kinematics(P1[0], P1[1], P1[2])
        M2 = robot.calculate_inv_kinematics(P2[0], P2[1], P2[2])
        M3 = robot.calculate_inv_kinematics(P3[0], P3[1], P3[2])
        
        P_s = [P0, P1, P2, P3]
        M_s = [M0, M1, M2, M3]
        
        for i in range(1):
            for j in range(4):
                self.x_val, self.y_val, self.z_val = P_s[j][0], P_s[j][1], P_s[j][2]
                self.x_input.setText(str(self.x_val))
                self.y_input.setText(str(self.y_val))
                self.z_input.setText(str(self.z_val))
                
                self.m1 = M_s[j][0]
                self.m2 = M_s[j][1]
                self.m3 = M_s[j][2]
                self.m1_input.setText(str(self.m1))
                self.m2_input.setText(str(self.m2))
                self.m3_input.setText(str(self.m3))
                
                if j == 3:
                    send_command(ardMega, m1 =self.m1 , m2 = self.m2, m3 = self.m3)
                    time.sleep(1)
                    send_command(ardMega, gripper = 1)
                    time.sleep(0.5)
                    
                else:
                    send_command(ardMega, m1 =self.m1 , m2 = self.m2, m3 = self.m3, gripper = 0)
                    time.sleep(0.5)
                    
        send_command(ardMega, m1 = 0, m2 = 0, m3 = 0, gripper = 1)
    
    def on_pnp_test_path(self):
        object_points = np.array([[0, 100, -500], [100, -100, -500], [200, 0, -500]])
        green_box_location = np.array([100, -230, -500])
        
        test_path = PNPTestPath(robot, object_points, green_box_location)
        commands, full_path = test_path.plan_path()
        
        for command in commands:
            M0 = command[0]
            M1 = command[1]
            M2 = command[2]
            
            self.m1 = M0
            self.m2 = M1
            self.m3 = M2
            
            self.m1_input.setText(str(self.m1))
            self.m2_input.setText(str(self.m2))
            self.m3_input.setText(str(self.m3))
            
            send_command(ardMega, m1 = self.m1, m2 = self.m2, m3 = self.m3)
            
            P0 = robot.calculate_fwd_kinematics(self.m1, self.m2, self.m3)[0]
            P1 = robot.calculate_fwd_kinematics(self.m1, self.m2, self.m3)[1]
            P2 = robot.calculate_fwd_kinematics(self.m1, self.m2, self.m3)[2]
            
            self.x_val = P0
            self.y_val = P1
            self.z_val = P2
            
            self.x_input.setText(str(self.x_val))
            self.y_input.setText(str(self.y_val))
            self.z_input.setText(str(self.z_val))
            # time.sleep(0.2)
            
    def update_visualization(self, m1, m2, m3):
        robot.calculate_fwd_kinematics(m1, m2, m3)
        fig = plot_delta_robot(robot.B1, robot.B2, robot.B3, robot.WP, robot.r, robot.s)
        plot_div = plot(fig , output_type = 'div', include_plotlyjs = 'cdn')
        self.web_engine_view.setHtml(plot_div)
        
    def move_to_pos(self):
        if self.mode == 'Move To Position' and self.positioning_mode == 'Exact':
            x = int(float(self.x_input.text()))
            y = int(float(self.y_input.text()))
            z = int(float(self.z_input.text()))
            
            self.m1, self.m2, self.m3 = robot.calculate_inv_kinematics(x, y, z)
            send_command(ardMega, m1 = self.m1, m2 = self.m2, m3 = self.m3)
            
            self.m1_input.setText(str(self.m1))
            self.m2_input.setText(str(self.m2))
            self.m3_input.setText(str(self.m3))
            
            self.update_visualization(self.m1, self.m2, self.m3)
        
    def move_to_angles(self):
        if self.mode == 'Move Joints' and self.positioning_mode == 'Exact':
            self.m1 = int(float(self.m1_input.text()))
            self.m2 = int(float(self.m2_input.text()))
            self.m3 = int(float(self.m3_input.text()))
            send_command(ardMega, m1 = self.m1, m2 = self.m2, m3 = self.m3)
            
            self.x_val, self.y_val, self.z_val = robot.calculate_fwd_kinematics(self.m1, self.m2, self.m3)
            self.x_input.setText(str(self.x_val))
            self.y_input.setText(str(self.y_val))
            self.z_input.setText(str(self.z_val))
                
            self.update_visualization(self.m1, self.m2, self.m3)
    
    def zero_m1_motor(self):
        send_command(ardMega, calibrate_motor= 4)
        self.m1 = 0
        self.m1_input.setText(str(self.m1))
        self.update_visualization(self.m1, self.m2, self.m3)
    
    def zero_m2_motor(self):
        send_command(ardMega, calibrate_motor= 5)
        self.m2 = 0 
        self.m2_input.setText(str(self.m2))
        self.update_visualization(self.m1, self.m2, self.m3)
            
    def zero_m3_motor(self):
        send_command(ardMega, calibrate_motor= 6)
        self.m3 = 0
        self.m3_input.setText(str(self.m3))
        self.update_visualization(self.m1, self.m2, self.m3)
    
    def on_back_to_homepage(self):
        self.go_back.emit()
            
class TomatoSortingPage(QWidget):
    go_back = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.initUI()
        self.robot = robot
        self.conveyor_robot = ConveyorRobot(self.robot, z_offset = 100)
        self.camera = cv2.VideoCapture(2)

    def initUI(self):
        self.captureTimer = QTimer(self)
        self.captureTimer.timeout.connect(self.capture_and_process_frame)
        
        # Layouts
        mainLayout = QVBoxLayout()
        controlsLayout = QHBoxLayout()
        displayLayout = QHBoxLayout()

        # Controls
        self.conveyorSpeedSlider = QSlider(Qt.Horizontal)
        self.conveyorSpeedSlider.setMinimum(10)
        self.conveyorSpeedSlider.setMaximum(100)
        self.conveyorSpeedSlider.setValue(50)
        self.conveyorSpeedSlider.setTickPosition(QSlider.TicksBelow)
        self.conveyorSpeedSlider.setTickInterval(10)

        self.frameRateSpinBox = QSpinBox()
        self.frameRateSpinBox.setMinimum(1)
        self.frameRateSpinBox.setMaximum(30)
        self.frameRateSpinBox.setValue(4)

        self.objectsCountSpinBox = QSpinBox()
        self.objectsCountSpinBox.setMinimum(1)
        self.objectsCountSpinBox.setMaximum(10)
        self.objectsCountSpinBox.setValue(3)

        self.startButton = QPushButton("Start")
        self.startButton.clicked.connect(self.startProcessing)

        # Adding controls to layout
        controlsLayout.addWidget(QLabel("Conveyor Speed:"))
        controlsLayout.addWidget(self.conveyorSpeedSlider)
        controlsLayout.addWidget(QLabel("Frame Rate (fps):"))
        controlsLayout.addWidget(self.frameRateSpinBox)
        controlsLayout.addWidget(QLabel("Objects to Pick:"))
        controlsLayout.addWidget(self.objectsCountSpinBox)
        controlsLayout.addWidget(self.startButton)

        # Display Labels
        self.liveFeedLabel = QLabel()
        self.processedFrameLabel = QLabel()

        displayLayout.addWidget(self.liveFeedLabel)
        displayLayout.addWidget(self.processedFrameLabel)

        # Main Layout
        mainLayout.addLayout(controlsLayout)
        mainLayout.addLayout(displayLayout)
        self.setLayout(mainLayout)

    
    def capture_and_process_frame(self):
        ret, frame = self.camera.read()
        if ret:
            self.display_frame(self.liveFeedLabel, frame)

            processed_frame = self.process_frame(frame)

            self.display_frame(self.processedFrasmeLabel, processed_frame)
    
    def process_frame(self, frame):
        # Process the frame using ImageProcessor
        image_processor = ImageProcessor(frame)
        object_centers = image_processor.detect_objects()

        # Draw rectangles around detected objects
        processed_frame = image_processor.draw_rectangles()

        # Select a specified number of objects based on the user input
        num_objects_to_pick = self.objectsCountSpinBox.value()
        if len(object_centers) > num_objects_to_pick:
            selected_objects = random.sample(object_centers, num_objects_to_pick)
        else:
            selected_objects = object_centers

        # # Adjust for conveyor movement
        # conveyor_speed_y = self.conveyorSpeedSlider.value()  # Get conveyor speed from the slider
        # adjusted_robot_coordinates = []
        # for object_center in selected_objects:
        #     robot_center = image2Dworld3D(object_center)
        #     time_to_reach = 1  # Time to reach the object from the current position
        #     adjusted_robot_center = adjust_for_conveyor_movement(robot_center, time_to_reach, conveyor_speed_y)
        #     adjusted_robot_coordinates.append(adjusted_robot_center)


        # box_points = [[-200, 230, -500], [200, -230, -500], [-300, 230, -500]]
        # for i, adjusted_robot_center in enumerate(adjusted_robot_coordinates):
        #     # Select the corresponding box point for the object
        #     box_point = box_points[i % len(box_points)]

        #     # Generate spline path for pick-and-place operation
        #     pick_path = self.conveyor_robot.create_spline_path(adjusted_robot_center, box_point)
        #     place_path = self.conveyor_robot.create_spline_path(box_point, adjusted_robot_center)

        #     # For each point on the path, calculate the inverse kinematics and send commands to the robot
        #     for point in pick_path + place_path:
        #         print(point)
        #         m1, m2, m3 = robot.calculate_inv_kinematics(point[0], point[1], point[2])
        #         send_command(ardMega, m1, m2, m3)  # ser should be the serial connection to your robot's controller

        # # Return the processed frame to be displayed
        return processed_frame
    
    def display_frame(self, label, frame):
        # Convert frame to QPixmap and display it
        qformat = QImage.Format_Indexed8 if len(frame.shape) == 2 else QImage.Format_RGB888
        out_image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], qformat)
        out_image = out_image.rgbSwapped()
        label.setPixmap(QPixmap.fromImage(out_image))
        label.setScaledContents(True)
    
    def startProcessing(self):
        if self.startButton.text() == "Start":
            if not self.camera.isOpened():
                self.camera = cv2.VideoCapture(2)  
            
            fps = self.frameRateSpinBox.value()
            self.captureTimer.start(int(1000 / fps))  

            self.startButton.setText("Stop")
        else:
            self.captureTimer.stop()
            if self.camera.isOpened():
                self.camera.release()

            self.startButton.setText("Start")
        
    def closeEvent(self, event):
        if self.captureTimer.isActive():
            self.captureTimer.stop()
        
        if self.camera.isOpened():
            self.camera.release()
        
        event.accept()  


        
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Stacked widget to hold the different pages
        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        # Create the pages
        self.robot_connection_page = RobotConnection()
        self.calibration_page = CalibrationPage()
        self.home_page = HomePage()
        self.manual_operation_page = ManualOperationPage()
        self.tomato_sorting_page = TomatoSortingPage()

        # Add pages to the stacked widget
        self.stacked_widget.addWidget(self.robot_connection_page)
        self.stacked_widget.addWidget(self.calibration_page)
        self.stacked_widget.addWidget(self.home_page)
        self.stacked_widget.addWidget(self.manual_operation_page)
        self.stacked_widget.addWidget(self.tomato_sorting_page)

        # Connect the signal to the slot to change pages
        self.robot_connection_page.connection_successful.connect(self.display_calibration_page)
        self.calibration_page.calibration_complete.connect(self.display_homepage)
        self.calibration_page.skip_calibration.connect(self.display_homepage)
        self.manual_operation_page.go_back.connect(self.display_homepage)
        self.tomato_sorting_page.go_back.connect(self.display_homepage)
        self.home_page.manual_op_req.connect(self.display_manual_operation_page)
        self.home_page.tomato_sorting_req.connect(self.display_tomato_sorting_page)
        self.home_page.back_to_calibration.connect(self.display_calibration_page)

    def display_homepage(self):
        self.stacked_widget.setCurrentWidget(self.home_page)
        
    def display_manual_operation_page(self):
        self.stacked_widget.setCurrentWidget(self.manual_operation_page)
    
    def display_calibration_page(self):
        self.stacked_widget.setCurrentWidget(self.calibration_page)
    
    def display_tomato_sorting_page(self):
        self.stacked_widget.setCurrentWidget(self.tomato_sorting_page)

def main():
    app = QApplication(sys.argv)
    # Change fontsizes for pushbuttons, labels and radio buttons
    # app.setStyleSheet("QPushButton {font-size: 20px; background-color: }"
    #                   "QRadioButton {font-size: 20px;}"
    #                   "QLabel {font-size: 20px;}")
    app.setStyleSheet("""
            QWidget {
                background-color: #f0f0f0; /* Light grey background for the page */
            }

            QPushButton {
                padding: 5px;
                font-size: 18px;
                border: 1px solid #9e9e9e; /* Slightly darker border for more contrast */
                border-radius: 15px; /* Rounded corners for buttons */
                background-color: #d6e4ff; /* More saturated blue */
                color: #0050b3; /* Darker blue text for more contrast */
            }

            QPushButton#primary {
                background-color: #4098d7; /* More saturated primary blue */
                color: white;
            }

            QPushButton#secondary {
                background-color: #f2f2f2; /* Light grey for less important buttons */
                color: #333; /* Darker text for contrast */
            }

            QPushButton#gripperOpen {
                background-color: #32c787; /* Saturated green for OPEN state */
                color: white;
            }

            QPushButton#gripperClose {
                background-color: #ff6b6b; /* Saturated red for CLOSE state */
                color: white;
            }

            QLabel {
                font-size: 20px;
                border-radius: 10px; /* Rounded corners for a softer look */
                padding: 5px;
                background-color: #f2f2f2; /* Soft light gray background */
                color: #333; /* Dark text for contrast */
                border: 1px solid #dcdcdc; /* Light border to maintain some definition */
            }

            QRadioButton {
                font-size: 18px;
                color: #333;
                padding: 5px;
            }

            QRadioButton::indicator {
                width: 15px;
                height: 15px;
            }

            QRadioButton::indicator::unchecked {
                background-color: #f0f0f0;
                border: 1px solid #9e9e9e;
                border-radius: 7px; /* Half of width and height to make it circular */
            }

            QRadioButton::indicator::checked {
                background-color: #4098d7; /* Match primary button color */
                border: 1px solid #9e9e9e;
                border-radius: 7px;
            }
            QPushButton#plusButton {
                background-color: #4CAF50; /* Green background for plus */
                color: white; /* White text for readability */
                border-radius: 15px; /* Rounded corners */
                font-weight: bold; /* Make the plus symbol more prominent */
            }

            QPushButton#minusButton {
                background-color: #f44336; /* Red background for minus */
                color: white; /* White text for readability */
                border-radius: 15px; /* Rounded corners */
                font-weight: bold; /* Make the minus symbol more prominent */
            }
            QLineEdit {
                font-size: 18px;
                color: #333; /* Text color */
                padding: 5px;
                border: 1px solid #9e9e9e; /* Solid border for definition */
                border-radius: 15px; /* Rounded corners */
                background-color: #ffffff; /* White background */
                selection-background-color: #4CAF50; /* Color when text is selected */
                selection-color: white; /* Text color when selected */
            }
        """)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
