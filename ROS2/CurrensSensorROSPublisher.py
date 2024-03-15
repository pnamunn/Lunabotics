import serial
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
#from diagnostic_msgs import KeyValue.msg

#Service node will recieve an error message from the client
#will print out thier is a drop in current 
#will wait three seconds and then send an aknoledgement

class CurrentSensorPublisher(Node):
    def __init__(self):
        super().__init__('CurrentPublisher')
        #create a publisher node of type= KeyValue, topic= current, callback= current_callback, 
        self.publisher_ = self.create_publisher(String, 'current', 10)
        
        self.get_logger().info("Pubber node has been created")
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self): #only publish if a problem arrises
        #TODO fix Serial
        #ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1) #TODO find out of this is correct port
        msg = String()
        msg.data = 'LeftBack'                               #name of wheel with problem
        #request.value = 0                           
        volts = 1
        if volts > 1: #if less than an amp of current alert 
            msg = ('Problem with wheel "%s", current sesnor is reading "%s"volts.' %msg.data %volts)
            self.publisher_.publish(msg)
            self.get_logger().info(msg)


        #time.wait(1)

def main(args=None):
    rclpy.init(args=args)           #get rclpy library
    pubber = CurrentSensorPublisher()      #create a node


    rclpy.spin(pubber)            #run until it is killed


    pubber.destroy_node()         #free node
    rclpy.shutdown()               #stop RoS


if __name__ == '__main__':
    main()
