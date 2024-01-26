
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

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)      # for Nano currently on the Zyn mobile


    def send(self, cmd):        # Used to serial write ASCII cmds
        self.ser.write(cmd.encode())


    def joy_callback(self, msg):
        ''' Callback function grabs some of the values being published by /joy topic, converts
        them to ASCII values, and sends serially to Arduino. '''

        self.button_values = msg.buttons
        self.get_logger().info(f'Subber received buttons = {self.button_values}')

        self.axes_values = msg.axes
        self.get_logger().info(f'Subber received axes = {self.axes_values}')

        # Used to toggle between using Joysticks (0) and Dpad (1)
        self.control_toggle = 0
        

        ''' Motor control using the Joysticks '''
        # if self.control_toggle == 0:
        if (self.axes_values[7] == 0 and self.axes_values[6] == 0):   # if Left Joystick is in neutral
            pass
        elif (self.axes_values[7] == 1):    # moving Left Joystick up
            self.send('w')
        elif (self.axes_values[7] == -1):    # moving Left Joystick down
            self.send('s')
    




        ''' Motor control using the Dpad '''
        # if self.control_toggle == 1:
        if (self.axes_values[1] == -0.0 and self.axes_values[0] == -0.0):      # No presses on Dpad occurring
            if (self.button_values[4] == 1):      # LB pressed
                self.send('1')        # retract
            elif (self.button_values[6] == 1):    # LT pressed
                self.send('3')      # extend
            else:
                self.send('m')        # send stop by sending an unused ASCII value

        elif (self.axes_values[1] == 1.0):      # Dpad up is pressed
            self.send('w')        # move forward
        elif (self.axes_values[1] == -1.0):      # Dpad down is pressed
             self.send('s')        # move backward
        elif (self.axes_values[0] == 1.0):      # Dpad left is pressed
            self.send('a')        # skid steer left
        elif (self.axes_values[0] == -1.0):      # Dpad right is pressed
            self.send('d')        # skid steer right


        ''' Depo bin controls '''
        if (self.button_values[X] == 1):    # RB pressed
            self.send('j')      # tilt up
        elif (self.button_values[X] == 1):    # RT pressed
            self.send('l')      # tilt down




def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = GamepadSubber()    # creates a node instance

    rclpy.spin(subber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()
    self.ser.close()


if __name__ == '__main__':
    main()
