import rclpy                    # ROS2-Python API lib
from rclpy.node import Node     # lib for node creation

class GamepadPublisher(Node):
    '''Child class of Node'''
    def __init__(self):
        super().__init__('gamepad_publisher')

        # creates publisher that publishes msg type=
        self.pubber = self.create_publisher(<msg type publisher will publish>, 
                                               <topicName the publisher will publish to>)
        
        # will publish every 0.5 sec
        TIMER_PERIOD = 0.5

        # creates timer
        self.timer = self.create_time(TIMER_PERIOD)

        self.i = 0


    def timer_callback(self):
        '''This function called every 0.5 sec'''

        msg = 
        
        # publishes msg to the pubber's topic
        self.pubber.publish(msg)

        # prints msg to terminal
        self.get_logger().info(f"Publishing: {msg}")

        self.i += 1


    def main():
        # init that rclpy lib
        rclpy.init()

        # creates a node
        gamepad_publisher = GamepadPublisher()

        # spins node (endlessly loops)
        rclpy.spin(gamepad_publisher)

        gamepad_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()