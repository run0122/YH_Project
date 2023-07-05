from PySide2.QtWidgets import *
from PySide2.QtCore import *
from PySide2.QtGui import *

import sys
import rospy
from std_msgs.msg import String

class MainWindow(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)

        self.button1 = QPushButton(self)
        self.button1.setIcon(QIcon("./res/Marlboro.jpg"))
        self.button1.setIconSize(QSize(200,200))
        self.button1.clicked.connect(self.on_button1_clicked)

        self.button2 = QPushButton(self)
        self.button2.setIcon(QIcon("./res/IceVolt.jpg"))
        self.button2.setIconSize(QSize(200,200))
        self.button2.clicked.connect(self.on_button2_clicked)

        self.button3 = QPushButton(self)
        self.button3.setIcon(QIcon("./res/Mevius.jpg"))
        self.button3.setIconSize(QSize(200,200))
        self.button3.clicked.connect(self.on_button3_clicked)

        self.itemInfo = QLabel(self)
        self.itemInfo.setText("Default")
        self.itemInfo.setFixedSize(200,200)

        self.runButton = QPushButton("출발", self)
        self.runButton.setFixedSize(50,50)
        self.runButton.clicked.connect(self.on_runButton_clicked)

        imageLayout = QHBoxLayout()
        imageLayout.addWidget(self.button1)
        imageLayout.addWidget(self.button2)
        imageLayout.addWidget(self.button3)

        itemAndButtonLayout = QHBoxLayout()
        itemAndButtonLayout.addWidget(self.itemInfo)
        itemAndButtonLayout.addWidget(self.runButton)
        
        layout = QVBoxLayout()
        layout.addLayout(imageLayout)
        layout.addLayout(itemAndButtonLayout)
        
        self.setLayout(layout)

    def on_button1_clicked(self):
        self.itemInfo.setText("Marlboro")
        self.itemNumber = '1'

    def on_button2_clicked(self):
        self.itemInfo.setText("IceVolt")
        self.itemNumber = '2'

    def on_button3_clicked(self):
        self.itemInfo.setText("FrenchBlack")
        self.itemNumber = '3'

    def on_runButton_clicked(self):
        pub = rospy.Publisher('/gui_info', String, queue_size=10)
        pub.publish(self.itemNumber)
        

def prepare():
    rospy.init_node('gui')
    
if __name__ == '__main__':

    prepare()

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    app.exec_()
