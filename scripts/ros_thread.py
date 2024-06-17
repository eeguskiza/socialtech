from PyQt5.QtCore import QThread

class RosThread(QThread):
    def __init__(self, route_function, route_index):
        super().__init__()
        self.route_function = route_function
        self.route_index = route_index

    def run(self):
        try:
            self.route_function(self.route_index)
        except Exception as e:
            print(f"Error while running the route: {e}")
