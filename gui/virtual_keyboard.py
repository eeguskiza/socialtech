from PyQt5.QtWidgets import QMainWindow, QPushButton, QGridLayout, QWidget, QLineEdit, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt

class VirtualKeyboard(QMainWindow):
    def __init__(self, input_field):
        super().__init__()
        self.input_field = input_field
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Virtual Keyboard")
        self.setGeometry(100, 100, 800, 400)
        self.setStyleSheet("background-color: lightgray;")

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        
        self.layout = QVBoxLayout(self.central_widget)
        
        # Display input field content
        self.display_label = QLabel(self.input_field.text(), self)
        self.display_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.display_label)
        
        self.grid_layout = QGridLayout()
        
        positions = [(i, j) for i in range(4) for j in range(10)]
        buttons = [
            '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',
            'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p',
            'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';',
            'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/'
        ]

        for position, key in zip(positions, buttons):
            button = QPushButton(key)
            button.setFixedSize(60, 60)
            button.setStyleSheet("font-size: 18pt;")
            button.clicked.connect(lambda _, k=key: self.key_pressed(k))
            self.grid_layout.addWidget(button, *position)

        space_button = QPushButton('Space')
        space_button.setFixedSize(60 * 6, 60)
        space_button.setStyleSheet("font-size: 18pt;")
        space_button.clicked.connect(lambda: self.key_pressed(' '))
        self.grid_layout.addWidget(space_button, 4, 2, 1, 6)
        
        self.layout.addLayout(self.grid_layout)

    def key_pressed(self, key):
        self.input_field.setText(self.input_field.text() + key)
        self.display_label.setText(self.input_field.text())

    def center(self):
        frame_geom = self.frameGeometry()
        cp = self.screen().availableGeometry().center()
        frame_geom.moveCenter(cp)
        self.move(frame_geom.topLeft())
