import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QPushButton,
                             QVBoxLayout, QHBoxLayout, QGridLayout, QLineEdit, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage

from image_pro import ImageProcessor
import numpy as np
import cv2

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Main window setup
        self.setWindowTitle('Delta Robot Control Interface')
        self.setGeometry(100, 100, 1024, 768)  # Adjust size as needed

        # Create central widget and layout
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)


        # Detected Tomatoes and their Coordinates
        detected_tomatoes_layout = QHBoxLayout()
        self.tomato_frames = []
        self.coordinate_inputs = []

        for i in range(3):
            tomato_layout = QVBoxLayout()

            # Tomato image placeholder
            tomato_display = QLabel()
            tomato_display.setFrameStyle(QFrame.Panel | QFrame.Sunken)
            tomato_display.setPixmap(QPixmap('path/to/tomato_placeholder.png'))  # Placeholder image
            tomato_display.setMinimumSize(160, 120)  # Adjust size as needed
            tomato_layout.addWidget(tomato_display)
            self.tomato_frames.append(tomato_display)

            # Coordinate inputs for each tomato
            coord_layout = QGridLayout()
            coord_layout.addWidget(QLabel('X'), 0, 0)
            x_input = QLineEdit()
            coord_layout.addWidget(x_input, 0, 1)
            coord_layout.addWidget(QLabel('Y'), 1, 0)
            y_input = QLineEdit()
            coord_layout.addWidget(y_input, 1, 1)
            coord_layout.addWidget(QLabel('Z'), 2, 0)
            z_input = QLineEdit()
            coord_layout.addWidget(z_input, 2, 1)

            tomato_layout.addLayout(coord_layout)
            self.coordinate_inputs.append((x_input, y_input, z_input))

            detected_tomatoes_layout.addLayout(tomato_layout)

        main_layout.addLayout(detected_tomatoes_layout)

        # Live Feed Area
        live_feed_layout = QHBoxLayout()
        self.live_feed_display = QLabel()  # Placeholder for the live feed
        self.live_feed_display.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.live_feed_display.setPixmap(QPixmap('path/to/live_feed_placeholder.png'))  # Placeholder image
        self.live_feed_display.setMinimumSize(640, 480)  # Adjust size as needed
        live_feed_layout.addWidget(self.live_feed_display)
        main_layout.addLayout(live_feed_layout)

        # Bottom Area for Control Panel and Image Processing Steps
        bottom_layout = QHBoxLayout()

        # Control Panel (CP) Area
        cp_area = QWidget()
        cp_layout = QVBoxLayout(cp_area)

        # Operation Buttons
        self.start_button = QPushButton('Start')
        cp_layout.addWidget(self.start_button)

        self.stop_button = QPushButton('Stop')
        cp_layout.addWidget(self.stop_button)

        # Status Display
        self.status_display = QLabel('Status: Idle')
        cp_layout.addWidget(self.status_display)

        bottom_layout.addWidget(cp_area)

        # Image Processing Step Display
        self.image_processing_display = QLabel()
        self.image_processing_display.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.image_processing_display.setPixmap(QPixmap('path/to/processing_step_placeholder.png'))  # Placeholder image
        self.image_processing_display.setMinimumSize(320, 240)  # Adjust size as needed
        bottom_layout.addWidget(self.image_processing_display)

        main_layout.addLayout(bottom_layout)
        
        placeholder_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.image_processor = ImageProcessor(placeholder_image)

        # Connect buttons to processing functions
        self.start_button.clicked.connect(self.start_image_processing)
        self.stop_button.clicked.connect(self.stop_robot)

    # Placeholder methods for various functionalities
    def start_robot(self):
        pass

    def stop_robot(self):
        pass

    def update_live_feed(self):
        pass

    def update_image_processing_steps(self):
        pass

    def update_detected_tomatoes(self):
        pass

    def update_coordinates(self):
        pass

    def update_status(self, status):
        self.status_display.setText(f'Status: {status}')
    
    def start_image_processing(self):
        # Here you would get the actual BGR image from your camera feed
        bgr_image = self.get_camera_image()
        self.image_processor.img = bgr_image
        self.image_processor.org_img = bgr_image.copy()
        
        # Apply the image processing steps
        self.image_processor.apply_watershed()

        # Update GUI with the processed images
        self.update_image_display()

    def update_image_display(self):
        # Update the live feed (if needed)
        self.update_live_feed()

        # Update the detected tomatoes images
        for i, info in enumerate(self.image_processor.cropped_objects_info[:3]):
            q_img = self.convert_cv_qt(info['cropped_image'])
            self.tomato_frames[i].setPixmap(QPixmap.fromImage(q_img))

        # Update the image processing steps display
        if self.image_processor.processing_steps_images:
            q_img = self.convert_cv_qt(self.image_processor.processing_steps_images[-1])
            self.image_processing_display.setPixmap(QPixmap.fromImage(q_img))

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        return convert_to_Qt_format.scaled(self.image_processing_display.width(), self.image_processing_display.height(), Qt.KeepAspectRatio)

    def get_camera_image(self):
        # This function should capture an image from your camera feed
        # For now, it returns a placeholder image
        return np.zeros((480, 640, 3), dtype=np.uint8)

# Main function to run the application
def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
