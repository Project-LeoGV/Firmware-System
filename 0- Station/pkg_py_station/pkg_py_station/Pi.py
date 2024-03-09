#!/usr/bin/env python3
import rclpy
from  rclpy.node import Node
from  std_msgs.msg import String

class RaspberryPiNode(Node):

    def __init__(self):        
        super().__init__("node_pi")
        self.get_logger().info("Pi Node Has been started")
    
        self.ESCs_subscriper_ = self.create_subscription(String, "ESCs",self.ESCs_callback, 10) #Topic for controlling ESC
        self.stepper_subscriper_ = self.create_subscription(String, "Stepper",self.stepper_callback, 10) #Topic for Controlling Stepper Motors
        self.sensors_publisher_ = self.create_publisher(String, "Sensors", 10) #Topic for Feedbacking 
        
        self.BMS_publisher_ = self.create_publisher(String,"BMS",10)
        self.timer = self.create_timer(0.5,self.BMS_callback)
        self.get_logger().info("BMS has been started")
        
    def ESCs_callback(self,msg):
        #self.Serial_()
        #self.get_logger().info("ESC is receiving: {}".format(msg.data))
        #self.ser.write(msg.encode('ascii'))
        pass
    
    def stepper_callback(self,msg):
        self.get_logger().info("Stepper is receiving: {}".format(msg.data))

    def BMS_callback(self):
        msg = String()
        msg.data = "0010412350000\n"
        self.get_logger().info("BMS is receiving: {}".format(msg.data))
        self.BMS_publisher_.publish(msg)
'''
    def Serial_(self):
        print("Welcome To USB")
        print("Enter The number of The port")
        # com = "/dev/ttyUSB" + input()           # Linux
        self.com = input("com")                   # Windows
        self.ser = serial.Serial(port=self.com, baudrate=115200)
'''

def main(args=None): 
    rclpy.init(args=args)
    node = RaspberryPiNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()
