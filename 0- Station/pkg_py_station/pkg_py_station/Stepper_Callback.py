from std_msgs.msg import String

def stepperControl(self,buttonID):
        msg = String()
        if(buttonID == "up"):
            msg.data = "0200402500000"                  
        elif(buttonID == "down"):
            msg.data = "0200401200000"
        elif(buttonID == "rotateright"):
            msg.data = "0300401200000"
        elif(buttonID == "rotateleft"):           
            msg.data = "0400407700000"
        self.plates_publisher.publish(msg)