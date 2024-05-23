from PyQt5.QtWidgets import QMainWindow, QPushButton, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import os
from gui.login_window import LoginWindow

class WelcomeWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Socialtech - Welcome")
        self.setGeometry(100, 100, 1024, 600)

        # Set background image
        self.label = QLabel(self)
        self.label.setGeometry(0, 0, 1024, 600)
        #pixmap = QPixmap(os.path.join(os.path.dirname(__file__), '../assets/mainMenu.webp'))
        #self.label.setPixmap(pixmap)
        #Fondo azul pastel
        self.label.setStyleSheet("background-color: #AED6F1")
        self.label.setScaledContents(True)

        # Add start button bold text
        self.start_button = QPushButton("Comenzar", self)
        self.start_button.setStyleSheet("font: bold 20pt")
        self.start_button.setGeometry(412, 400, 200, 50)
        self.start_button.clicked.connect(self.start_button_clicked)

    def start_button_clicked(self):
        self.login_window = LoginWindow()
        self.login_window.show()
        self.close()
