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
    def __init__(self):
        self.robot_name = "LeoGV"
        self.node = rclpy.create_node("node_station")
        self.subscription = self.node.create_subscription(String, 'BMS', self.callback, 10)
        
        self.motors_publisher = self.node.create_publisher(String,"ESCs",10)
        self.plates_publisher = self.node.create_publisher(String,"Stepper",10)

    def callback(self, msg):
        print("Received message:", msg.data)
        self.node.get_logger().info(msg.data)
        self.signal.update_label.emit(msg.data)
        

    def run(self, signal):
        self.signal = signal
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            rclpy.shutdown()

    def stepperControl(self,buttonID):
        msg = String()
        if(buttonID == "up"):
            msg.data = "0010402500000"                 #change with correct msg
        elif(buttonID == "down"):
            msg.data = "0030401200000"
        self.plates_publisher.publish(msg)

    def ODriveControl(self,buttonID):
        msg = String()
        if(buttonID == "forward"):
            msg.data = "0020525010000"
        elif(buttonID == "reverse"):
            msg.data = "0060400910000"
        self.motors_publisher.publish(msg)

    #def node_publisher_(self, text):
    #    msg = String()
    #    msg.data = 'Forward: ' + text['Forward'] +'Backward: ' + text['Backward']
    #    self.plates_publisher.publish(msg)

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
