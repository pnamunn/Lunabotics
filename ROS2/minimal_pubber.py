''' This code will publish Howdy every 0.5 seconds '''


import rclpy                    # ROS2-Python API lib
from rclpy.node import Node     # lib for node creation
from std_msgs.msg String




class Pubber(Node):
    '''Child class of Node'''
    def __init__(self):
        super().__init__('pubber')  # inits the attributes of the parent class, Node

        # creates publisher that publishes msg_type=String to topicName='test_topic', QoS=10
        self.pubber = self.create_publisher(String, 'test_topic', 10)
        self.get_logger().info("Pubber node instance created")
        
        TIMER_PERIOD = 0.5
        self.timer = self.create_timer(TIMER_PERIOD)    # creates timer to flag every 0.5 seconds

        self.i = 0


    def timer_callback(self):
        '''This function is called every 0.5 sec & publishes msgs'''
        msg = String()

        msg.data = "Howdy %d" % self.i      # creates msg data that will be sent to pubber's topic (test_topic)

        self.pubber.publish(msg)        # publishes msg data to the pubber's topic (test_topic)

        self.get_logger().info(f"Publishing: {msg.data}")    # prints msg data to terminal, just so you can see it

        self.i += 1




def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    pubber = Pubber()    # creates a node instance

    rclpy.spin(pubber)       # spins node (endlessly loops) until the user kills the node program (Ctrl+C)

    pubber.destroy_node()   
    rclpy.shutdown()




if __name__ == '__main__':
    main()