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
        
        #Controls
        #self.Controls = {'Forward': "False",'Backward': "False",'Left': "False",'Right': "False",'Stepper_Lift_Up': "False",'Stepper_Lift_Down': "False"}
        #self.ui.ForwardButton.pressed.connect(lambda: self.Control_Handler(self.Controls, 'Forward', "True"))
        #self.ui.ReverseButton.pressed.connect(lambda: self.Control_Handler(self.Controls, 'Backward', "True"))
        #self.ui.UpButton.pressed.connect(lambda: self.Control_Handler(self.Controls, 'Stepper_Lift_Up', "True"))
        
        #and so on
        
        #for releasing button to handle removing it
        #self.ui.ForwardButton.released.connect(lambda: self.Control_Handler(self.Controls, 'Forward', "False"))
        #self.ui.ReverseButton.released.connect(lambda: self.Control_Handler(self.Controls, 'Backward', "False"))
        #self.ui.UpButton.released.connect(lambda: self.Control_Handler(self.Controls, 'Stepper_Lift_Up', "False"))

        #and so on
        
        #signal for emmiting Controls
        self.signal = Signal()
        #self.signal.Publish_message.connect(lambda: self.node.stepperControl(self.get_key(self.Controls)))
        
        # Controlling stepper motors (plate)
        self.ui.UpButton.clicked.connect(lambda: self.node.stepperControl("up"))
        self.ui.DownButton.clicked.connect(lambda: self.node.stepperControl("down"))
        
        # Controlling ODrive
        self.ui.ForwardButton.clicked.connect(lambda: self.node.ODriveControl("forward"))
        self.ui.ReverseButton.clicked.connect(lambda: self.node.ODriveControl("reverse"))

            
    
        
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
        self.ui.Motor1CurrentVlaue.setText(text)
        
