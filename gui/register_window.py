from PyQt5.QtWidgets import QMainWindow, QPushButton, QLabel, QLineEdit, QVBoxLayout, QWidget, QHBoxLayout, QSpacerItem, QSizePolicy, QMessageBox
from PyQt5.QtCore import Qt
import sqlite3

class RegisterWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Socialtech - Register")
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

        # Add confirm password input
        self.confirm_password_label = QLabel("Confirm Password:", self)
        self.confirm_password_label.setStyleSheet("font-size: 14pt")
        self.left_layout.addWidget(self.confirm_password_label, alignment=Qt.AlignCenter)

        self.confirm_password_input = QLineEdit(self)
        self.confirm_password_input.setEchoMode(QLineEdit.Password)
        self.confirm_password_input.setStyleSheet("font-size: 14pt")
        self.confirm_password_input.setFixedWidth(200)
        self.left_layout.addWidget(self.confirm_password_input, alignment=Qt.AlignCenter)

        # Add register button
        self.register_button = QPushButton("Register", self)
        self.register_button.setStyleSheet("font-weight: bold; font-size: 14pt")
        self.register_button.setFixedWidth(200)
        self.register_button.clicked.connect(self.register)
        self.left_layout.addWidget(self.register_button, alignment=Qt.AlignCenter)

        # Spacer at the bottom
        self.bottom_spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.left_layout.addItem(self.bottom_spacer)

        # Add layouts to main layout
        self.main_layout.addLayout(self.left_layout)

    def register(self):
        username = self.username_input.text()
        password = self.password_input.text()
        confirm_password = self.confirm_password_input.text()

        if password != confirm_password:
            QMessageBox.warning(self, "Registration Failed", "Passwords do not match")
            return

        if self.add_user_to_db(username, password):
            QMessageBox.information(self, "Registration Successful", "User registered successfully")
            self.close()
        else:
            QMessageBox.warning(self, "Registration Failed", "Username already exists")

    def add_user_to_db(self, username, password):
        conn = sqlite3.connect('database/robot_data.db')
        cursor = conn.cursor()

        try:
            cursor.execute('''
            INSERT INTO user (username, password) VALUES (?, ?)
            ''', (username, password))
            conn.commit()
            return True
        except sqlite3.IntegrityError:
            return False
        finally:
            conn.close()
