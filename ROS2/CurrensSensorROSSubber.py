import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String

#from diagnostic_msgs import KeyValue.msg
#ommon_interfaces/diagnostic_msgs

#Service node will recieve an error message from the client
#will print out thier is a drop in current 
#will wait three seconds and then send an aknoledgement

class CurrentSensorSubscriber(Node):
    def __init__(self):
        super().__init__('CurrentSubscriber')
        #create a publisher node of type= KeyValue, topic= current, callback= current_callback, 
        self.subscriber_ = self.create_subscription(String, 'current',self.current_callback, 10)
        self.get_logger().info("Pubber node has been created")


    def current_callback(self, msg): #only publish if a problem arrises  
        self.get_logger().info(msg)



def main(args=None):
    rclpy.init(args=args)           #get rclpy library
    pubber = CurrentSensorSubscriber()      #create a node


    rclpy.spin(pubber)            #run until it is killed


    pubber.destroy_node()         #free node
    rclpy.shutdown()               #stop RoS


if __name__ == '__main__':
    main()
