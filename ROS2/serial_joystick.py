''' Creates a node that converts gamepad input to rover motor commands. '''

''' Takes in the 32 bit float values being published by /joy topic & converts 
them to 8 bit ASCII values.  Then sends ASCII data serially to Arduino Nano. '''

''' To use this code: Run joy_node on Linux laptop to publish /joy topic.
Then run this code on the Jetson to create a gamepad_subber_node that will sub to the
/joy topic.  Logitech F310 gamepad must be flipped to D mode & have the Mode button light on. '''


import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Node that subs to /joy pubber
class GamepadSubber(Node):

    def __init__(self):     # Constructor

        super().__init__('gamepad_subber_node')  

        ''' Creates a subber node of msg_type=Joy, topic_name=joy, callback_function=joy_callback() '''
        self.subber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info("GamepadSubber(Node) instance created")

        # self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)      # for Nano currently on the Zyn mobile

        self.deadzone = 0.2
          


    # def send(self, cmd):        # Used to serial write ASCII cmds
    #     if (type(cmd) == str):
    #         self.ser.write(cmd.encode())
    #     elif (type(cmd) == bytes):
    #         self.ser.write(cmd)


    def joystick_math(self, x, y):
        self.max = max(abs(y), abs(x))
        self.sum = y + x
        self.diff = y - x

        if y >= 0:      # if y is positive
            if x >= 0:      # if x is positive      # Quadrant 1
                self.left_motor = self.max
                self.right_motor = self.diff
            else:       # if x is negative          # Quadrant 2
                self.left_motor = self.sum
                self.right_motor = self.max

        else:   # if y is negative                  # Quadrant 4
            if x >= 0:      # if x is positive
                self.left_motor = self.sum
                self.right_motor = -(self.max)
            else:       # if x is negative          # Quadrant 3
                self.left_motor = -(self.max)
                self.right_motor = self.diff

        self.get_logger().info(f'When X = {self.x}   Y = {self.y}')   
        self.get_logger().info(f'Left Motor = {self.left_motor}')
        self.get_logger().info(f'Right Motor = {self.right_motor}')


    def joy_callback(self, msg):
        ''' Callback function grabs some of the values being published by /joy topic, converts
        them to ASCII values, and sends serially to Arduino. '''

        self.button_values = msg.buttons
        self.get_logger().info(f'Subber received buttons = {self.button_values}')

        self.axes_values = msg.axes
        self.get_logger().info(f'Subber received axes = {self.axes_values}')
        

        ''' Motor control using the Joysticks '''
    
        if (self.axes_values[0] > self.deadzone or self.axes_values[0] < -(self.deadzone) or self.axes_values[1] > self.deadzone or self.axes_values[1] < -(self.deadzone)):   # if Left Joystick is in neutral
            self.joystick_math(self.axes_values[0], self.axes_values[1])
        else:       # if left joystick is inside of deadzone
            self.get_logger().info(f'In deadzone')




def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = GamepadSubber()    # creates a node instance

    rclpy.spin(subber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()
    # self.ser.close()


if __name__ == '__main__':
    main()
