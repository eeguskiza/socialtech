from PyQt5.QtWidgets import QMainWindow, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QListWidget, QListWidgetItem, QStackedWidget, QSpacerItem, QSizePolicy, QPushButton
from PyQt5.QtCore import Qt
from gui.default_routes_window import DefaultRoutesWindow

class MainMenuWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Socialtech - Main Menu")
        self.setGeometry(100, 100, 1024, 600)

        # Main widget
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        # Main layout
        self.main_layout = QHBoxLayout(self.central_widget)

        # Sidebar layout
        self.sidebar = QListWidget(self)
        self.sidebar.setFixedWidth(200)
        self.sidebar.setStyleSheet("""
            QListWidget {
                font-size: 16pt;
                font-weight: bold;
                text-align: center;
            }
            QListWidget::item {
                height: 50px;
                padding: 10px;
            }
        """)
        self.sidebar.addItem(QListWidgetItem("Rutas Disponibles"))
        self.sidebar.addItem(QListWidgetItem("Crea tu ruta"))
        self.sidebar.addItem(QListWidgetItem("Cuadros"))
        self.sidebar.addItem(QListWidgetItem("Mapa"))
        self.sidebar.addItem(QListWidgetItem("Cerrar Sesión"))
        self.sidebar.addItem(QListWidgetItem("Ajustes"))
        self.sidebar.currentRowChanged.connect(self.display_content)

        # Add spacer to center the sidebar items vertically
        self.sidebar_layout = QVBoxLayout()
        self.sidebar_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        self.sidebar_layout.addWidget(self.sidebar)
        self.sidebar_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        sidebar_container = QWidget()
        sidebar_container.setLayout(self.sidebar_layout)

        # Stack for different pages
        self.stack = QStackedWidget(self)
        self.option_messages = [
            "Estas son las rutas disponibles para su selección.",
            "Aquí puedes crear tu propia ruta personalizada.",
            "Visualiza los cuadros de mando y estadísticas.",
            "Accede al mapa interactivo.",
            "Cierra la sesión actual y vuelve a la pantalla de inicio de sesión.",
            "Ajusta las configuraciones de la aplicación."
        ]
        
        for i, message in enumerate(self.option_messages):
            page = QWidget()
            layout = QVBoxLayout(page)
            label = QLabel(f"Opción - {i + 1}", self)
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label)

            # Add introductory message
            intro_message = QLabel(message, self)
            intro_message.setAlignment(Qt.AlignCenter)
            layout.addWidget(intro_message)

            # Add "¡Vamos!" button
            vamos_button = QPushButton("¡Vamos!", self)
            vamos_button.setFixedSize(100, 50)
            vamos_button.clicked.connect(lambda _, i=i: self.navigate_to_option(i + 1))
            layout.addWidget(vamos_button, alignment=Qt.AlignCenter)

            self.stack.addWidget(page)

        # Add sidebar and stack to main layout
        self.main_layout.addWidget(sidebar_container)
        self.main_layout.addWidget(self.stack)

    def display_content(self, index):
        self.stack.setCurrentIndex(index)

    def navigate_to_option(self, option_index):
        if option_index == 1:
            self.default_routes_window = DefaultRoutesWindow()
            self.default_routes_window.show()
        else:
            print(f"Redirigiendo a la ventana para Opción - {option_index}")
            # Aquí puedes añadir la lógica para abrir la nueva ventana correspondiente a la opción seleccionada
