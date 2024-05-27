import sys
from PyQt5.QtWidgets import QApplication
from gui.welcome_window import WelcomeWindow

def main():
    app = QApplication(sys.argv)
    window = WelcomeWindow()
    window.show()
    window.move(app.desktop().screen().rect().center() - window.rect().center())
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
