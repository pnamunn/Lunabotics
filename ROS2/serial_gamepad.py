
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

   #     self.get_logger().info("GamepadSubber(Node) instance created")

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)      # for Nano currently on the Zyn mobile


    def send(self, cmd):
        self.ser.write(cmd.encode())


    def joy_callback(self, msg):
        ''' Callback grabs some of the values being published by /joy topic '''

        self.button_values = msg.buttons
 #1       self.get_logger().info(f'Subber received buttons = {self.button_values}')

        self.axes_values = msg.axes
#        self.get_logger().info(f'Subber received axes = {self.axes_values}')


        ''' Sends serial ascii letters (this also serves to change the axes float 32 bit value to an 8 bit value) '''
        
        # Controls Dpad forward & backward
        if (self.axes_values[7] == -0.0):      # No presses on Dpad up or down
           self.send('m')        # send stop by sending an unused ASCII value
        elif (self.axes_values[7] == 1.0):      # Dpad up is pressed
            self.send('w')        # move forward
        elif (self.axes_values[7] == -1.0):      # Dpad down is pressed
             self.send('s')        # move backward

        # # Controls Dpad left & right
        # if (self.axes_values[0] == -0.0):      # No presses on Dpad left or right
        #     self.ser.write(b'm')        # send stop by sending an unused ASCII value
        # elif (self.axes_values[0] == 1.0):      # Dpad left is pressed
        #     self.ser.write(b'a')        # skid steer left
        # elif (self.axes_values[0] == -1.0):      # Dpad right is pressed
        #     self.ser.write(b'd')        # skid steer right


        # # Controls buttons for excavation subsystem

        # # controls linear actuator extend
        # if (self.button_values[4]):      # LB pressed
        #     self.ser.write(b'U')        # extend linear actuators
        # else:
        #     # self.ser.write(b'm')        # send stop by sending an unused ASCII value
        #     pass

        # # controls linear actuator retract
        # if (self.axes_values[2] < 1.0):    # LT pressed
        #     self.ser.write(b'D')


def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = GamepadSubber()    # creates a node instance

    rclpy.spin(subber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()

    self.ser.close()


if __name__ == '__main__':
    main()
