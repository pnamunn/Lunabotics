''' Creates a node that converts gamepad input to rover motor commands. '''

''' Takes in the 32 bit float values being published by /joy topic & converts 
them to 8 bit ASCII values.  Then sends ASCII data serially to Arduino Nano. '''

''' To use this code: Run joy_node on Linux laptop to publish /joy topic.
Then run this code on the Jetson to create a gamepad_subber_node that will sub to the
/joy topic.  Logitech F310 gamepad must be flipped to D mode & have the Mode button light OFF. '''


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

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)      # serial to Arduino Mega 

        self._deadzone = 0.2
        self.curr_joy = [0, 0]
        self.last_joy = [0, 0]

        self.left_motor = 0
        self.right_motor = 0



    def send(self, cmd):        # Used to serial write ASCII cmds
        if (type(cmd) == str):
            self.ser.write(cmd.encode())
        elif (type(cmd) == bytes):
            self.ser.write(cmd)


    def arcade_drive_math(self, x, y):
        x = -x      # Change bc gamepad's x axes are backwards
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


        ''' Normalize motor values for the Arduino's 16 bit duty cycle values '''
        self.left_motor = int( (self.left_motor * 1000) + 3000 )
        self.right_motor = int( (self.right_motor * 1000) + 3000 ) 

        self.get_logger().info(f'When X = {x}   Y = {y}')   
        self.get_logger().info(f'Left Motor = {self.left_motor}')
        self.get_logger().info(f'Right Motor = {self.right_motor}')


        ''' Sends the motor's duty cycle values to the Arduino '''
        self.send("2")     # first send message_type
        self.send(self.right_motor & 0b0000_1111)   # send right_motor low
        self.send(self.right_motor >> 8)            # send right_motor high
        self.send(self.left_motor & 0b0000_1111)   # send left_motor low
        self.send(self.left_motor >> 8)            # send left_motor high
        
        self.last_joy[0] = self.left_motor
        self.last_joy[1] = self.right_motor
        


    def joy_callback(self, msg):
        ''' Callback function grabs some of the values being published by /joy topic, converts
        them to ASCII values, and sends serially to Arduino. '''

        self.button_values = msg.buttons
        self.get_logger().info(f'Subber received buttons = {self.button_values}')

        self.axes_values = msg.axes
        self.get_logger().info(f'Subber received axes = {self.axes_values}')
        

        ''' Motor control using the Joysticks '''
        # If left joystick is outside of deadzone
        if (self.axes_values[0] > self._deadzone or self.axes_values[0] < -(self._deadzone) or self.axes_values[1] > self._deadzone or self.axes_values[1] < -(self._deadzone)):
            self.arcade_drive_math(self.axes_values[0], self.axes_values[1])    # calc left and right motor values

        # if left joystick is within deadzone
        else:       
            self.get_logger().info(f'In deadzone')
            self.curr_joy[0] = self.left_motor
            self.curr_joy[1] = self.right_motor

            if (self.curr_joy != self.last_joy):
                self.send("m")

            self.last_joy[0] = self.left_motor
            self.last_joy[1] = self.right_motor



        
   



def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = GamepadSubber()    # creates a node instance

    rclpy.spin(subber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()
    # self.ser.close()


if __name__ == '__main__':
    main()
