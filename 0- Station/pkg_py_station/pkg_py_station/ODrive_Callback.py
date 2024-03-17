from std_msgs.msg import String

def ODriveControl(self,buttonID):
        msg = String()
        #TODO: input vel msg with different input vel values +ve value for forward and -ve for reverse
        if(buttonID == "forward"):
            msg.data = "00d0825010000"                       
        elif(buttonID == "reverse"):
            msg.data = "00d0800210000"
        elif(buttonID == "right"):
            msg.data = "00d0800110000"
        elif(buttonID == "left"):
            msg.data = "00d0800910000"
        self.motors_publisher.publish(msg)