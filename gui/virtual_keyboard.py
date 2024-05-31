from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit, QGridLayout
from PyQt5.QtCore import Qt

class VirtualKeyboard(QDialog):
    def __init__(self, input_field):
        super().__init__()
        self.input_field = input_field
        self.setWindowTitle("Teclado Virtual")
        self.setGeometry(100, 100, 1024, 600)

        
        # Centro la ventana en la pantalla
        self.center()

        layout = QVBoxLayout()

        # Campo de texto para mostrar lo que se está escribiendo
        self.display_field = QLineEdit(self)
        self.display_field.setReadOnly(True)
        self.display_field.setText(input_field.text())
        layout.addWidget(self.display_field)

        keys_layout = QGridLayout()
        keys = [
            ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],
            ['q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p'],
            ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l'],
            ['z', 'x', 'c', 'v', 'b', 'n', 'm']
        ]

        for i, row in enumerate(keys):
            for j, key in enumerate(row):
                button = QPushButton(key)
                button.setFixedSize(60, 60)
                button.clicked.connect(lambda _, k=key: self.key_pressed(k))
                keys_layout.addWidget(button, i, j)

        layout.addLayout(keys_layout)

        control_layout = QHBoxLayout()
        space_button = QPushButton('Space')
        space_button.setFixedSize(460, 60)
        space_button.clicked.connect(self.space_pressed)
        control_layout.addWidget(space_button)

        backspace_button = QPushButton('Backspace')
        backspace_button.setFixedSize(120, 60)
        backspace_button.clicked.connect(self.backspace_pressed)
        control_layout.addWidget(backspace_button)

        enter_button = QPushButton('Enter')
        enter_button.setFixedSize(120, 60)
        enter_button.clicked.connect(self.enter_pressed)
        control_layout.addWidget(enter_button)

        layout.addLayout(control_layout)

        self.setLayout(layout)

    def key_pressed(self, key):
        self.display_field.setText(self.display_field.text() + key)
        self.input_field.setText(self.display_field.text())

    def space_pressed(self):
        self.display_field.setText(self.display_field.text() + ' ')
        self.input_field.setText(self.display_field.text())

    def backspace_pressed(self):
        self.display_field.setText(self.display_field.text()[:-1])
        self.input_field.setText(self.display_field.text())

    def enter_pressed(self):
        self.accept()

    def center(self):
        qr = self.frameGeometry()
        cp = self.screen().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
