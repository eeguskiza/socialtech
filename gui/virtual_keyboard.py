from PyQt5.QtWidgets import QWidget, QGridLayout, QPushButton, QLineEdit, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import Qt

class VirtualKeyboard(QWidget):
    def __init__(self, input_field):
        super().__init__()
        self.input_field = input_field
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Virtual Keyboard")
        self.setGeometry(100, 100, 600, 300)
        
        # Main layout
        layout = QVBoxLayout()

        # Input field display
        self.display = QLineEdit(self)
        self.display.setReadOnly(True)
        layout.addWidget(self.display)
        
        # Keyboard layout
        grid_layout = QGridLayout()
        keys = [             ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],             ['q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p'],             ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l'],             ['z', 'x', 'c', 'v', 'b', 'n', 'm']         ]

        positions = [(i, j) for i in range(5) for j in range(10)]
        
        for position, key in zip(positions, buttons):
            button = QPushButton(key)
            button.clicked.connect(lambda _, k=key: self.key_pressed(k))
            grid_layout.addWidget(button, *position)

        layout.addLayout(grid_layout)
        self.setLayout(layout)
        self.center()

    def key_pressed(self, key):
        if key == 'Space':
            self.input_field.setText(self.input_field.text() + ' ')
            self.display.setText(self.display.text() + ' ')
        elif key == 'Backspace':
            self.input_field.setText(self.input_field.text()[:-1])
            self.display.setText(self.display.text()[:-1])
        elif key == 'Enter':
            self.input_field.setText(self.display.text())
            self.close()
        else:
            self.input_field.setText(self.input_field.text() + key)
            self.display.setText(self.display.text() + key)

    def center(self):
        screen_geometry = self.screen().availableGeometry()
        cp = screen_geometry.center()
        frame_geometry = self.frameGeometry()
        frame_geometry.moveCenter(cp)
        self.move(frame_geometry.topLeft())
