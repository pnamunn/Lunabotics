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
        super().__init__('current service')
        #create a publisher node of type= KeyValue, topic= current, callback= current_callback, 
        self.publisher_ = self.create_publisher(String, 'current',current_callback, 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)


    def current_callback(self, request, response): #only publish if a problem arrises
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1) #TODO find out of this is correct port
        request.key = 'LeftBack'                               #name of wheel with problem
        request.value = 0                           

        if ser < 1: #if less than an amp of current alert 
            request.value = ser
            msg = ('Problem with wheel "%s", currrent is "%s".' %request.key %request.value)
            self.publisher_.publish(msg)
            self.get_logger().info(msg)
        time.wait(1)

def main(args=None):
    rclpy.init(args=args)           #get rclpy library
    pubber = CurrentSensorPublisher()      #create a node


    rclpy.spin(pubber)            #run until it is killed


    pubber.destroy_node()         #free node
    rclpy.shutdown()               #stop RoS


if __name__ ==
