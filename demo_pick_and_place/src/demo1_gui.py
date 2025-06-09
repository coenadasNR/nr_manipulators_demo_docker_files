#!/usr/bin/env python3

import sys
import shlex
import subprocess
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget

class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NR Manipulation Project Demo 2 GUI")
        self.setGeometry(100, 100, 300, 150)

        # Store the subprocess handle
        self.process = None

        # GUI layout
        layout = QVBoxLayout()

        # Start button
        self.button_start = QPushButton("Start", self)
        self.button_start.clicked.connect(self.start_launch)
        layout.addWidget(self.button_start)

        # Stop button
        self.button_stop = QPushButton("Stop", self)
        self.button_stop.clicked.connect(self.stop_launch)
        layout.addWidget(self.button_stop)

        # Set the layout to a central widget
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def start_launch(self):
        # Only start if not already running
        if self.process is None:
            cmd = "ros2 launch xarm_moveit_config xarm5_linear_moveit_fake.launch.py"
            args = shlex.split(cmd)
            self.process = subprocess.Popen(args)

    def stop_launch(self):
        # Terminate the launch process if running
        if self.process is not None:
            self.process.terminate()
            self.process.wait()
            self.process = None

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = RobotGUI()
    gui.show()
    sys.exit(app.exec_())
