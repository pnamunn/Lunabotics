''' Example code for learning '''
''' This code will subscribe to ___ '''


import rclpy                    # ROS2-Python API lib
from rclpy.node import Node     # lib for node creation
from std_msgs.msg import String     # import data type String
# view all available data types by typing in CLI ros2 interface list



''' Creates an object class for a Subscriber node '''
class Subber(Node):     # is a child class of Node

    def __init__(self):     # Constructor
        super().__init__('subber')      # names your node subber when its constructed

        # creates a subscriber that can subscribe to msg_type=String, from the topic topicName='example_topic',
        # uses the callback function listening_callback(), and has QoS(queue_size)=10
        self.subber = self.create_subscription(String, 'example_topic', self.listening_callback, 10)

        self.get_logger().info("Subber node instance created")


    def listening_callback(self, msg):
        ''' Prints the String msg it received from its attached pubber '''

        self.get_logger().info(f"Subber received: {msg.data}")    # prints msg data to terminal, just so you can see it



def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = Subber()    # creates a node instance

    rclpy.spin(subber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()


if __name__ == '__main__':
    main()
