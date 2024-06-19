from PyQt5.QtWidgets import QMainWindow, QPushButton, QLabel, QLineEdit, QVBoxLayout, QWidget, QHBoxLayout, QSpacerItem, QSizePolicy, QMessageBox
from PyQt5.QtCore import Qt
import sqlite3
from utils import center_window
from .virtual_keyboard import VirtualKeyboard
from .main_menu_window import MainMenuWindow  
from .register_window import RegisterWindow  

class LoginWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Socialtech - Login")
        self.setGeometry(100, 100, 1024, 600)
        center_window(self)

        # Set background color
        self.setStyleSheet("background-color: #003B73;")

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
        self.username_label.setStyleSheet("font-size: 20pt; color: white;")
        self.left_layout.addWidget(self.username_label, alignment=Qt.AlignCenter)

        self.username_input = QLineEdit(self)
        self.username_input.setStyleSheet("font-size: 20pt")
        self.username_input.setFixedWidth(300)
        self.left_layout.addWidget(self.username_input, alignment=Qt.AlignCenter)

        # Add password input
        self.password_label = QLabel("Contraseña:", self)
        self.password_label.setStyleSheet("font-size: 20pt; color: white;")
        self.left_layout.addWidget(self.password_label, alignment=Qt.AlignCenter)

        self.password_input = QLineEdit(self)
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setStyleSheet("font-size: 20pt")
        self.password_input.setFixedWidth(300)
        self.left_layout.addWidget(self.password_input, alignment=Qt.AlignCenter)

        # Add buttons
        self.login_button = QPushButton("LOGIN", self)
        self.login_button.setStyleSheet("font-weight: bold; font-size: 20pt; height: 50px; width: 200px;")
        self.login_button.clicked.connect(self.login)
        self.left_layout.addWidget(self.login_button, alignment=Qt.AlignCenter)

        self.register_button = QPushButton("REGISTRASE", self)
        self.register_button.setStyleSheet("font-weight: bold; font-size: 20pt; height: 50px; width: 200px;")
        self.register_button.clicked.connect(self.open_register_window)
        self.left_layout.addWidget(self.register_button, alignment=Qt.AlignCenter)

        self.exit_button = QPushButton("SALIR", self)
        self.exit_button.setStyleSheet("font-weight: bold; font-size: 20pt; height: 50px; width: 200px;")
        self.exit_button.clicked.connect(self.close)
        self.left_layout.addWidget(self.exit_button, alignment=Qt.AlignCenter)

        # Spacer at the bottom
        self.bottom_spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.left_layout.addItem(self.bottom_spacer)

        # Add layouts to main layout
        self.main_layout.addLayout(self.left_layout)

        # Conectar eventos de clic a los campos de entrada
        self.username_input.mousePressEvent = lambda event: self.show_keyboard(self.username_input)
        self.password_input.mousePressEvent = lambda event: self.show_keyboard(self.password_input)

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

    def show_keyboard(self, input_field):
        self.keyboard = VirtualKeyboard(input_field)
        self.keyboard.show()
