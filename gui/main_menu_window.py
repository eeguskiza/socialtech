from PyQt5.QtWidgets import QMainWindow, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QListWidget, QListWidgetItem, QStackedWidget, QSpacerItem, QSizePolicy, QPushButton, QComboBox
from PyQt5.QtCore import Qt
from .default_routes_window import DefaultRoutesWindow
from .custom_route_window import CustomRouteWindow
from utils import center_window

class MainMenuWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Socialtech - Main Menu")
        self.setGeometry(100, 100, 1024, 600)
        center_window(self)

        # Main widget
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        # Main layout
        self.main_layout = QHBoxLayout(self.central_widget)

        # Sidebar container
        self.sidebar_container = QWidget(self)
        self.sidebar_container.setFixedWidth(250)
        self.sidebar_layout = QVBoxLayout(self.sidebar_container)

        # Combo box for main menu options
        self.combo_box = QComboBox(self)
        self.combo_box.addItem("Rutas")
        self.combo_box.addItem("Cuadros")
        self.combo_box.addItem("Otros Ajustes")
        self.combo_box.setFixedHeight(40)
        self.combo_box.currentIndexChanged.connect(self.update_list)

        self.sidebar_layout.addWidget(self.combo_box)

        # List widget for sub-options
        self.sidebar_list = QListWidget(self)
        self.sidebar_list.setStyleSheet("""
            QListWidget {
                font-size: 14pt;
                font-weight: bold;
            }
            QListWidget::item {
                height: 30px;
                padding: 5px;
            }
        """)
        self.update_list(0)  # Initialize with the first set of options
        self.sidebar_list.currentRowChanged.connect(self.display_content)

        self.sidebar_layout.addWidget(self.sidebar_list)

        self.main_layout.addWidget(self.sidebar_container)

        # Main content area
        self.content_area = QWidget(self)
        self.content_layout = QVBoxLayout(self.content_area)
        
        self.option_label = QLabel("Seleccione una opción del menú", self)
        self.option_label.setAlignment(Qt.AlignCenter)
        self.content_layout.addWidget(self.option_label)
        
        self.intro_message = QLabel("", self)
        self.intro_message.setAlignment(Qt.AlignCenter)
        self.content_layout.addWidget(self.intro_message)
        
        self.vamos_button = QPushButton("¡Vamos!", self)
        self.vamos_button.setFixedSize(100, 50)
        self.vamos_button.clicked.connect(self.navigate_to_option)
        self.content_layout.addWidget(self.vamos_button, alignment=Qt.AlignCenter)

        self.main_layout.addWidget(self.content_area)

    def update_list(self, index):
        self.sidebar_list.clear()
        if index == 0:
            self.sidebar_list.addItem(QListWidgetItem("Rutas predeterminadas"))
            self.sidebar_list.addItem(QListWidgetItem("Crea tu ruta"))
            self.sidebar_list.addItem(QListWidgetItem("Mapa"))
        elif index == 1:
            self.sidebar_list.addItem(QListWidgetItem("Lista cuadros"))
            self.sidebar_list.addItem(QListWidgetItem("Info cuadros"))
            self.sidebar_list.addItem(QListWidgetItem("Asistente"))
        elif index == 2:
            self.sidebar_list.addItem(QListWidgetItem("Ajustes generales"))
            self.sidebar_list.addItem(QListWidgetItem("Cerrar sesión"))

    def display_content(self, index):
        current_item = self.sidebar_list.currentItem()
        if current_item:
            selected_option = current_item.text()
            self.option_label.setText(f"Opción - {index + 1}")
            if selected_option == "Rutas predeterminadas":
                self.intro_message.setText("Estas son las rutas disponibles para su selección.")
                self.vamos_button.setProperty("option", "rutas_predeterminadas")
            elif selected_option == "Crea tu ruta":
                self.intro_message.setText("Aquí puedes crear tu propia ruta personalizada.")
                self.vamos_button.setProperty("option", "crea_tu_ruta")
            elif selected_option == "Mapa":
                self.intro_message.setText("Accede al mapa interactivo.")
                self.vamos_button.setProperty("option", "mapa")
            elif selected_option == "Lista cuadros":
                self.intro_message.setText("Visualiza la lista de cuadros.")
                self.vamos_button.setProperty("option", "lista_cuadros")
            elif selected_option == "Info cuadros":
                self.intro_message.setText("Información sobre los cuadros.")
                self.vamos_button.setProperty("option", "info_cuadros")
            elif selected_option == "Asistente":
                self.intro_message.setText("Asistente para cuadros.")
                self.vamos_button.setProperty("option", "asistente")
            elif selected_option == "Ajustes generales":
                self.intro_message.setText("Ajusta las configuraciones generales de la aplicación.")
                self.vamos_button.setProperty("option", "ajustes_generales")
            elif selected_option == "Cerrar sesión":
                self.intro_message.setText("Cierra la sesión actual y vuelve a la pantalla de inicio de sesión.")
                self.vamos_button.setProperty("option", "cerrar_sesion")

    def navigate_to_option(self):
        selected_option = self.vamos_button.property("option")
        if selected_option == "rutas_predeterminadas":
            self.default_routes_window = DefaultRoutesWindow()
            self.default_routes_window.show()
        elif selected_option == "crea_tu_ruta":
            from gui.custom_route_window import CustomRouteWindow
            self.custom_route_window = CustomRouteWindow()
            self.custom_route_window.show()
        elif selected_option == "cerrar_sesion":
            print("Cerrando sesión")
            from gui.login_window import LoginWindow  # Importar aquí para evitar dependencias circulares
            self.login_window = LoginWindow()
            self.login_window.show()
            self.close()
        else:
            print(f"Ejecutando acción para la opción seleccionada: {selected_option}")
            # Aquí puedes añadir la lógica para abrir la nueva ventana correspondiente a la opción seleccionada
