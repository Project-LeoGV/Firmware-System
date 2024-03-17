from PyQt5.QtCore import QObject, pyqtSignal, QThread, Qt
from threading import Thread
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.executors import SingleThreadedExecutor
import sys


class Signal(QObject):
    update_label = pyqtSignal(str)
    Publish_message = pyqtSignal(dict)

class ROSListenerNode:
    from Stepper_Callback import stepperControl
    from ODrive_Callback import ODriveControl

    def __init__(self):
        self.robot_name = "LeoGV"
        self.node = rclpy.create_node("node_station")
        # Node publishers
        self.motors_publisher = self.node.create_publisher(String,"ESCs",10)
        self.plates_publisher = self.node.create_publisher(String,"Stepper",10)
        # Node subscribers
        self.plates_Subscriber = self.node.create_subscription(String,"Sensors",self.ODriveMotor1_Feedback,10)
        self.BMS_Subscriber = self.node.create_subscription(String, 'BMS', self.ODriveMotor2_Feedback, 10)
        
    def ODriveMotor1_Feedback(self,msg):
        print("ODrive motor1 has received feedback: ",msg.data)
        self.signal.update_label.emit(msg.data)
    
    def ODriveMotor2_Feedback(self,msg):
        print("ODrive motor2 has received feedback: ",msg.data)
        self.signal2.update_label.emit(msg.data)


    def BMS_Feedback(self):
        pass

    def run(self, signal):
        self.signal = signal
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            rclpy.shutdown()

def main(args=None):
    from GUI_Backend import LeoGUI
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    GUI_ = LeoGUI()
    
    signal = Signal()
    signal.update_label.connect(GUI_.update_gui) 
    
    ros_thread = Thread(target=ROSListenerNode().run, args=(signal,))
    ros_thread.start()
    
    GUI_.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
