''' Example code for learning '''
''' This code will publish Howdy every 0.5 seconds '''


import rclpy                    # ROS2-Python API lib
from rclpy.node import Node     # lib for node creation
from std_msgs.msg import String     # import data type String
# view all available data types by typing in CLI ros2 interface list



''' Creates an object class for a Publisher node '''
class Pubber(Node):     # is a child class of Node

    def __init__(self):     # Constructor
        super().__init__('pubber')  # names your node pubber when its constructed

        # creates a publisher that is made to publish msg_type=String, sent over the topic 
        # topicName='my_pubber_topic' and has QoS(queue_size)=10
        self.pubber = self.create_publisher(String, 'my_pubber_topic', 10)

        self.get_logger().info("Pubber node instance created")
        
        TIMER_PERIOD = 0.5
        self.timer = self.create_timer(TIMER_PERIOD)    # creates timer to flag every 0.5 seconds

        self.i = 0


    def timer_callback(self):
        '''This function is called every 0.5 sec & publishes msgs'''
        msg = String()

        msg.data = "Howdy %d" % self.i      # creates msg data that will be sent to pubber's topic (my_pubber_topic)

        self.pubber.publish(msg)        # publishes msg data to the pubber's topic (my_pubber_topic)

        self.get_logger().info(f"Publishing: {msg.data}")    # prints msg data to terminal, just so you can see it
        # if you eliminate this from your code, you can always see what a publisher is publishing by using ros2 topic echo </topic_name>

        self.i += 1




def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    pubber = Pubber()    # creates a node instance

    rclpy.spin(pubber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    pubber.destroy_node()   
    rclpy.shutdown()




if __name__ == '__main__':
    main()