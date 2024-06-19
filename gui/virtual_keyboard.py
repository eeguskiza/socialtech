from PyQt5.QtWidgets import QMainWindow, QPushButton, QGridLayout, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt

class VirtualKeyboard(QMainWindow):
    def __init__(self, input_field):
        super().__init__()
        self.input_field = input_field
        self.shift = False  # Track the state of the shift key
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
        self.display_label.setStyleSheet("font-size: 24pt;")  # Increase font size here
        self.layout.addWidget(self.display_label)
        
        self.grid_layout = QGridLayout()
        
        self.positions = [(i, j) for i in range(5) for j in range(10)]
        self.lowercase_buttons = [
            '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',
            'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p',
            'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', '✓',
            'z', 'x', 'c', 'v', 'b', 'n', 'm', 'Shift', '<--'
        ]
        self.uppercase_buttons = [
            '!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
            'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P',
            'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', '✓',
            'Z', 'X', 'C', 'V', 'B', 'N', 'M', 'Shift', '<--'
        ]
        self.create_buttons()

        space_button = QPushButton('Space')
        space_button.setFixedSize(60 * 6, 60)
        space_button.setStyleSheet("font-size: 18pt;")
        space_button.clicked.connect(lambda: self.key_pressed(' '))
        self.grid_layout.addWidget(space_button, 5, 2, 1, 6)
        
        self.layout.addLayout(self.grid_layout)

    def create_buttons(self):
        for position, key in zip(self.positions, self.lowercase_buttons if not self.shift else self.uppercase_buttons):
            button = QPushButton(key)
            button.setFixedSize(60, 60)
            button.setStyleSheet("font-size: 18pt;")
            if key == 'Shift':
                button.clicked.connect(self.toggle_shift)
            elif key == '✓':
                button.clicked.connect(self.enter_pressed)
            elif key == '<--':
                button.clicked.connect(self.backspace_pressed)
            else:
                button.clicked.connect(lambda _, k=key: self.key_pressed(k))
            self.grid_layout.addWidget(button, *position)

    def toggle_shift(self):
        self.shift = not self.shift
        self.update_buttons()

    def update_buttons(self):
        for i in range(len(self.positions)):
            button = self.grid_layout.itemAtPosition(*self.positions[i]).widget()
            key = self.lowercase_buttons[i] if not self.shift else self.uppercase_buttons[i]
            button.setText(key)
            if key == 'Shift':
                button.clicked.disconnect()
                button.clicked.connect(self.toggle_shift)
            elif key == '✓':
                button.clicked.disconnect()
                button.clicked.connect(self.enter_pressed)
            elif key == '<--':
                button.clicked.disconnect()
                button.clicked.connect(self.backspace_pressed)
            else:
                button.clicked.disconnect()
                button.clicked.connect(lambda _, k=key: self.key_pressed(k))

    def key_pressed(self, key):
        self.input_field.setText(self.input_field.text() + key)
        self.display_label.setText(self.input_field.text())

    def backspace_pressed(self):
        current_text = self.input_field.text()
        self.input_field.setText(current_text[:-1])
        self.display_label.setText(self.input_field.text())

    def enter_pressed(self):
        self.close()

    def center(self):
        frame_geom = self.frameGeometry()
        cp = self.screen().availableGeometry().center()
        frame_geom.moveCenter(cp)
        self.move(frame_geom.topLeft())
