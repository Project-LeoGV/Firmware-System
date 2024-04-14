#!usr/bin/env python3
import rclpy
from  rclpy.node import Node
from  std_msgs.msg import String

#DataType SerialCan
#from leogv_interfaces.msg import SerialCan
import serial



class RaspberryPiNode(Node):

    def __init__(self):        
        super().__init__("node_pi")
        
        self.ESCs_subscriper_ = self.create_subscription(String, "ESCs",self.ESCs_callback, 10)             # Topic for controlling ESC
        self.stepper_subscriper_ = self.create_subscription(String, "Stepper",self.stepper_callback, 10)    # Topic for Controlling Stepper Motors  
        
        self.temp = 0
        
        self.sensors_publisher_ = self.create_publisher(String, "Sensors", 10)  # Topic for Feedbacking Sensors 
        self.BMS_publisher_ = self.create_publisher(String,"BMS",10)            # Topic for Feedbacking BMS
        
        self.timer = self.create_timer(2, self.sensors_callback)
        
        self.get_logger().info("Pi Node Has started")
    
    def sensors_callback(self):
        print("Callback Called")
        request = "0291000000000"
        ser.write(request.encode('ascii'))
        Ack = ser.readline().decode()
        # Ack = "0000\n"
        message = String()
        
        if Ack == "0001\n":
            print("Sending...")
        elif Ack == "0002\n":
            print("Recciving...")
            message.data = ser.readline().decode()
        else:
            print("No Ack Received")
        print("reciving finished")
        # self.temp += 1
        # message.data = str(self.temp)
        self.sensors_publisher_.publish(message)
        
    def ESCs_callback(self, msg):
        message = msg.data    
        
        # Send and Receive Ack from USB
        ser.write(message.encode('ascii'))
        Ack = ser.readline().decode()
        if Ack == "0001\n":
            print("Sending...")
        elif Ack == "0002\n":
            print("Recciving...")
        else:
            print("No Ack Received")
            
        self.get_logger().info("ESC is receiving: {}".format(message))
        
    def stepper_callback(self, msg):
        message = msg.data
        
        # Send and Receive Ack from USB
        ser.write(message.encode('ascii'))
        Ack = ser.readline().decode()
        if Ack == "0001\n":
            print("Sending...")
        elif Ack == "0002\n":
            print("Recciving...")
        else:
            print("No Ack Received")   
        self.get_logger().info("Stepper is receiving: {}".format(message))
        

    
def main(args=None): 
    com = "/dev/ttyACM0"
    global ser
    ser = serial.Serial(port=com, baudrate=115200)
    
    rclpy.init(args=args)
    node = RaspberryPiNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    