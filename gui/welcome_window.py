from PyQt5.QtWidgets import QMainWindow, QPushButton, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import os
from gui.login_window import LoginWindow
from utils import center_window

class WelcomeWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Socialtech - Welcome")
        self.setGeometry(100, 100, 1024, 600)
        center_window(self)

        # Set background color to a darker blue
        self.label = QLabel(self)
        self.label.setGeometry(0, 0, 1024, 600)
        self.label.setStyleSheet("background-color: #003B73")  # Darker blue color
        self.label.setScaledContents(True)

        # Add the first logo
        self.logo1 = QLabel(self)
        self.logo1.setGeometry(50, -180, 688, 688)  # Adjust size and position as needed
        pixmap1 = QPixmap(os.path.join(os.path.dirname(__file__), '../assets/login.png'))
        self.logo1.setPixmap(pixmap1)
        self.logo1.setScaledContents(True)

        # Add the second logo
        self.logo2 = QLabel(self)
        self.logo2.setGeometry(774, 50, 200, 200)  # Adjust size and position as needed
        pixmap2 = QPixmap(os.path.join(os.path.dirname(__file__), '../assets/socialtech.png'))
        self.logo2.setPixmap(pixmap2)
        self.logo2.setScaledContents(True)

        # Add start button with larger size and bold text
        self.start_button = QPushButton("Comenzar", self)
        self.start_button.setStyleSheet("font: bold 24pt; height: 80px; width: 300px;")
        self.start_button.setGeometry(362, 400, 300, 80)
        self.start_button.clicked.connect(self.start_button_clicked)

    def start_button_clicked(self):
        self.login_window = LoginWindow()
        self.login_window.show()
        self.close()
