from PyQt5.QtWidgets import QMainWindow, QWidget, QGridLayout, QLabel
from PyQt5.QtCore import Qt
from scripts.ros_thread import RosThread
from utils import center_window
import scripts.ros_routes as ros_routes

class DefaultRoutesWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Socialtech - Rutas Disponibles")
        self.setGeometry(100, 100, 1024, 600)
        center_window(self)

        # Main widget
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        # Main layout
        self.main_layout = QGridLayout(self.central_widget)
        
        # Paneles para las rutas con nombres
        routes_info = [
            {"color": "#FF5733", "name": "Ordenada 0-6"},
            {"color": "#33FF57", "name": "Inversa 6-0"}
        ]
        
        for i, info in enumerate(routes_info):
            route_panel = QLabel(info["name"], self)
            route_panel.setStyleSheet(f"background-color: {info['color']}; font-size: 20pt; font-weight: bold; color: white;")
            route_panel.setAlignment(Qt.AlignCenter)
            route_panel.mousePressEvent = lambda event, idx=i: self.start_route(idx + 1)
            self.main_layout.addWidget(route_panel, i // 2, i % 2)
        
    def start_route(self, route_index):
        print(f"Iniciando ruta {route_index}")
        self.ros_thread = RosThread(ros_routes.follow_route, route_index)
        self.ros_thread.start()
