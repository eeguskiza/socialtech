import sys
from PyQt5.QtWidgets import QApplication
#import rospy
from gui.welcome_window import WelcomeWindow

def main():
    #rospy.init_node('route_follower', anonymous=True)
    app = QApplication(sys.argv)
    window = WelcomeWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
