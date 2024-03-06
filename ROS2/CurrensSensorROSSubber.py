import serial
import rclpy
from rclpy.node import Node
import time
from diagnostic_msgs import KeyValue.msg
#ommon_interfaces/diagnostic_msgs

#Service node will recieve an error message from the client
#will print out thier is a drop in current 
#will wait three seconds and then send an aknoledgement

class CurrentSensorService(Node):
    def __init__(self):
        super().__init__('current service')

        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.aknoledge_callback)

    def aknoledge_callback(self, request, response): #Code will wait 3 secods and then re
        response = true
        time.sleep(3)    
        self.get_logger().info('Current has fallen below the required limit, it is %f.6 amps', request.a)
        return response



def main(args=None):
    rclpy.init(args=args)           #get rclpy library
    sevice = CurrentSensorService()      #create a node


    rclpy.spin(service)            #run until it is killed


    service.destroy_node()         #free node
    rclpy.shutdown()               #stop RoS


if __name__ ==
