from PyQt5.QtWidgets import QMainWindow, QPushButton, QLabel, QLineEdit, QVBoxLayout, QWidget, QHBoxLayout, QSpacerItem, QSizePolicy, QMessageBox
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import os
import sqlite3
from gui.register_window import RegisterWindow
from gui.main_menu_window import MainMenuWindow

class LoginWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Socialtech - Login")
        self.setGeometry(100, 100, 1024, 600)

        # Main widget
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        # Main layout
        self.main_layout = QHBoxLayout(self.central_widget)

        # Left layout
        self.left_layout = QVBoxLayout()

        # Spacer at the top
        self.top_spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.left_layout.addItem(self.top_spacer)

        # Add username input
        self.username_label = QLabel("Username:", self)
        self.username_label.setStyleSheet("font-size: 14pt")
        self.left_layout.addWidget(self.username_label, alignment=Qt.AlignCenter)

        self.username_input = QLineEdit(self)
        self.username_input.setStyleSheet("font-size: 14pt")
        self.username_input.setFixedWidth(200)
        self.left_layout.addWidget(self.username_input, alignment=Qt.AlignCenter)

        # Add password input
        self.password_label = QLabel("Password:", self)
        self.password_label.setStyleSheet("font-size: 14pt")
        self.left_layout.addWidget(self.password_label, alignment=Qt.AlignCenter)

        self.password_input = QLineEdit(self)
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setStyleSheet("font-size: 14pt")
        self.password_input.setFixedWidth(200)
        self.left_layout.addWidget(self.password_input, alignment=Qt.AlignCenter)

        # Add buttons
        self.login_button = QPushButton("Login", self)
        self.login_button.setStyleSheet("font-weight: bold; font-size: 14pt")
        self.login_button.setFixedWidth(200)
        self.login_button.clicked.connect(self.login)
        self.left_layout.addWidget(self.login_button, alignment=Qt.AlignCenter)

        self.register_button = QPushButton("Register", self)
        self.register_button.setStyleSheet("font-weight: bold; font-size: 14pt")
        self.register_button.setFixedWidth(200)
        self.register_button.clicked.connect(self.open_register_window)
        self.left_layout.addWidget(self.register_button, alignment=Qt.AlignCenter)

        self.exit_button = QPushButton("Exit", self)
        self.exit_button.setStyleSheet("font-weight: bold; font-size: 14pt")
        self.exit_button.setFixedWidth(200)
        self.exit_button.clicked.connect(self.close)
        self.left_layout.addWidget(self.exit_button, alignment=Qt.AlignCenter)

        # Spacer at the bottom
        self.bottom_spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.left_layout.addItem(self.bottom_spacer)

        # Right layout
        self.right_layout = QVBoxLayout()

        # Add image
        self.image_label = QLabel(self)
        pixmap = QPixmap(os.path.join(os.path.dirname(__file__), '../assets/login.png'))
        pixmap = pixmap.scaled(512, 600, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image_label.setPixmap(pixmap)
        self.image_label.setScaledContents(True)
        self.right_layout.addWidget(self.image_label)

        # Add layouts to main layout
        self.main_layout.addLayout(self.left_layout)
        self.main_layout.addLayout(self.right_layout)

    def login(self):
        username = self.username_input.text()
        password = self.password_input.text()

        if self.verify_credentials(username, password):
            QMessageBox.information(self, "Login Successful", "Inicio de sesión correcto")
            self.open_main_menu_window()
        else:
            QMessageBox.warning(self, "Login Failed", "Usuario o contraseña incorrectos")

    def verify_credentials(self, username, password):
        conn = sqlite3.connect('database/robot_data.db')
        cursor = conn.cursor()

        cursor.execute('''
        SELECT * FROM user WHERE username=? AND password=?
        ''', (username, password))

        user = cursor.fetchone()
        conn.close()

        if user:
            return True
        return False

    def open_register_window(self):
        self.register_window = RegisterWindow()
        self.register_window.show()

    def open_main_menu_window(self):
        self.main_menu_window = MainMenuWindow()
        self.main_menu_window.show()
        self.close()
