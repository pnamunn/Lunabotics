''' Will take in the values being published by /joy topic & convert them
    to a smaller data size.  Then the data is serially sent to the Arduino. '''
    ''' /joy topic publishes a msg type of sensor_msgs/msg/Joy, inside which
        is packaged std_msgs/Header header,
                    float32[] axes,
                    int32[] buttons
    '''


import serial
import rclpy
import rclpy.node import Node
from geometry_msgs.msg import Twist

ser = serial.Serial('/dev/ttcyACM0', 9600, timeout=1)

# Node that subs to /joy pubber
class GamepadSubber(Node):

    def __init__(self):

        super().__init__('gamepad_subber_node')  

        ''' Creates a subber node of msg_type=Joy, topic_name=my_subber_topic, callback_function=joy_callback() '''
        self.subber = self.create_subscription(Joy, 'gamepad_subber_topic',self.joy_callback, 10)

        self.get_logger().info("GamepadSubber(Node) instance created")

        self.buttonA = 0

    def joy_callback(self, msg):
        ''' Callback grabs some of the values being published by /joy topic '''
        self.buttonA = buttons[0]
        self.get_logger().info(f'Subber received ButtonA = {self.buttonA}')


def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = GamepadValues()    # creates a node instance

    rclpy.spin(gamepad_values)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()


if __name__ == '__main__':
    main()