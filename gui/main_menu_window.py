from PyQt5.QtWidgets import QMainWindow, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QListWidget, QListWidgetItem, QStackedWidget, QSpacerItem, QSizePolicy
from PyQt5.QtCore import Qt

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
        for i in range(1, 7):
            page = QWidget()
            layout = QVBoxLayout(page)
            label = QLabel(f"Opción - {i}", self)
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label)
            self.stack.addWidget(page)

        # Add sidebar and stack to main layout
        self.main_layout.addWidget(sidebar_container)
        self.main_layout.addWidget(self.stack)

    def display_content(self, index):
        self.stack.setCurrentIndex(index)
