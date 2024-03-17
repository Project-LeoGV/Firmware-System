from GUI_FrontEnd import Ui_MainWindow
from camera import Worker_
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from Station import Signal, ROSListenerNode


class LeoGUI(QtWidgets.QMainWindow):

    def __init__(self):
        #Importing UI frontend window
        QtWidgets.QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.showMaximized()

        self.node = ROSListenerNode()
        
        #Camera
        self.Worker1 = Worker_()
        self.Worker1.start()
        self.Worker1.ImageUpdate.connect(self.ImageUpdateSlot)
        self.ui.RotateLeftButton.clicked.connect(self.CancelFeed)
        
        #signal for emmiting Controls
        self.signal = Signal()
        
        # Controlling stepper motors (plate)
        self.ui.UpButton.clicked.connect(lambda: self.node.stepperControl("up"))
        self.ui.DownButton.clicked.connect(lambda: self.node.stepperControl("down"))
        self.ui.RotateRightButton.clicked.connect(lambda: self.node.stepperControl("rotateright"))
        self.ui.RotateLeftButton.clicked.connect(lambda: self.node.stepperControl("rotateleft"))
        
        # Controlling ODrive
        self.ui.ForwardButton.clicked.connect(lambda: self.node.ODriveControl("forward"))
        self.ui.ReverseButton.clicked.connect(lambda: self.node.ODriveControl("reverse"))
        self.ui.RightButton.clicked.connect(lambda: self.node.ODriveControl("right"))
        self.ui.LeftButton.clicked.connect(lambda: self.node.ODriveControl("left"))



    #Camera Slot    
    def ImageUpdateSlot(self, Image):
        self.ui.label_28.setPixmap(QPixmap.fromImage(Image))
        
    def CancelFeed(self):
        self.Worker1.stop()
    
    def send__(self):
        pass
        
        
    #Control_Handler and emitter
    def Control_Handler(self, Controls, element, status):
        Controls[element] = status
        self.signal.Publish_message.emit(self.Controls)
    
    #GUI Reciever and Updater
    def update_gui(self, text):
        self.ui.Motor1CurrentVlaue.setText(text[5:9])
        self.ui.Motor1_RPM_Vlaue.setText(text[9:13])

    def update_motor2(self,text):
        self.ui.Motor2CurrentVlaue.setText(text[5:9])
        self.ui.Motor2_RPM_Vlaue.setText(text[9:13])

       
        
