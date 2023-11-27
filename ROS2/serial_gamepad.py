''' Will take in the values being published by /joy topic & convert them
    to a smaller data size.  Then the data is serially sent to the Arduino. '''
''' /joy topic publishes a msg type of sensor_msgs/msg/Joy, inside which
is packaged std_msgs/Header header, float32[] axes, int32[] buttons'''

''' What code is doing right now: subbing to /joy topic & printing to console the
gamepad's buttonA value'''
'''To have telecom gamepad control: Run joy_node on Linux laptop to pub /joy topic.
Then run this code on the Jetson create a gamepad_subber_node that will sub to the
/joy topic.'''


import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Node that subs to /joy pubber
class GamepadSubber(Node):

    def __init__(self):

        super().__init__('gamepad_subber_node')  

        ''' Creates a subber node of msg_type=Joy, topic_name=my_subber_topic, callback_function=joy_callback() '''
        self.subber = self.create_subscription(Joy, 'joy',self.joy_callback, 10)

        self.get_logger().info("GamepadSubber(Node) instance created")

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)      # for Nano currently on the Zyn mobile


    def joy_callback(self, msg):
        ''' Callback grabs some of the values being published by /joy topic '''

        self.button_values = msg.buttons
        self.get_logger().info(f'Subber received buttons = {self.button_values}')

        self.axes_values = msg.axes
        self.get_logger().info(f'Subber received axes = {self.axes_values}')

        # if-else logic
        # if button is pressed
        #   drive forward by sending ascii letter (this also serves to change the 32 bit button value to an 8 bit value)
        if (self.axes_values[1] == -0.0):      # No presses on Dpad north or south
            self.ser.write(b'm')        # send stop by sending an unused ASCII value
        elif (self.axes_values[1] == 1.0):      # Dpad north is pressed
            self.ser.write(b'w')        # move forward
        elif (self.axes_values[1] == -1.0):      # Dpad south is pressed
            self.ser.write(b's')        # move backward


def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = GamepadSubber()    # creates a node instance

    rclpy.spin(subber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()

    self.ser.close()


if __name__ == '__main__':
    main()