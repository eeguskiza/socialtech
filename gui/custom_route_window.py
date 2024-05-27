from PyQt5.QtWidgets import QMainWindow, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QPushButton, QMessageBox
from PyQt5.QtCore import Qt

class CustomRouteWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Socialtech - Crear tu Ruta")
        self.setGeometry(100, 100, 1024, 600)

        # Inicializar la lista de coordenadas
        self.coordinates = []

        # Main widget
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        # Main layout
        self.main_layout = QVBoxLayout(self.central_widget)

        self.info_label = QLabel("Seleccione los puntos en el orden que desee:", self)
        self.info_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.info_label)

        # Botones para seleccionar puntos
        self.buttons_layout = QHBoxLayout()
        self.buttons = []
        for i in range(6):
            btn = QPushButton(f"Punto {i + 1}", self)
            btn.setFixedSize(100, 50)
            btn.clicked.connect(lambda _, i=i: self.add_coordinate(i))
            self.buttons.append(btn)
            self.buttons_layout.addWidget(btn)

        self.main_layout.addLayout(self.buttons_layout)

        self.confirm_button = QPushButton("Confirmar Ruta", self)
        self.confirm_button.setFixedSize(150, 50)
        self.confirm_button.clicked.connect(self.confirm_route)
        self.main_layout.addWidget(self.confirm_button, alignment=Qt.AlignCenter)

    def add_coordinate(self, point_index):
        # Agregar la coordenada seleccionada a la lista
        self.coordinates.append([point_index * 10, point_index * 10])  # Coordenadas de ejemplo
        self.buttons[point_index].setEnabled(False)  # Deshabilitar el botón seleccionado

    def confirm_route(self):
        if len(self.coordinates) < 2:
            QMessageBox.warning(self, 'Error', 'Debe seleccionar al menos 2 puntos para confirmar la ruta.')
            return

        reply = QMessageBox.question(self, 'Confirmar Ruta',
                                     f"Ha seleccionado {len(self.coordinates)} puntos. ¿Está seguro de que desea confirmar esta ruta?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            # Llamar al script de ROS para ejecutar la ruta
            import scripts.execute_route as execute_route
            execute_route.execute_custom_route(self.coordinates)
        else:
            # Reiniciar la selección de la ruta
            self.coordinates = []
            for btn in self.buttons:
                btn.setEnabled(True)
