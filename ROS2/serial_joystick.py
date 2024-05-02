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
from bitarray import bitarray
import time


# Node that subs to /joy pubber
class GamepadSubber(Node):

    def __init__(self):     # Constructor

        super().__init__('gamepad_subber_node')  

        ''' Creates a subber node of msg_type=Joy, topic_name=joy, callback_function=joy_callback() '''
        self.subber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # TODO make cod autodetect correct port for Arduino Mega
        # self.usb_port = "/dev/tty/ACM0"
        # self.get_logger().info(f"GamepadSubber(Node) instance created.\nUsing {} for serial comm to Arduino.")

        self.get_logger().info("GamepadSubber(Node) instance created.")


        self.ser = serial.Serial('/dev/ttyACM1', 500000, bytesize=8, timeout=2)      # serial to Arduino Mega

        self._deadzone = 0.1
        self.curr_joy = [0, 0]  # holds left & right motor vals
        self.last_joy = [0, 0]  # holds left & right motor vals

        self.last_butt = bytearray([0, 0, 0, 0, 0, 0, 0, 0])   # holds self.button_values array

        self.left_motor = 0
        self.right_motor = 0



    def send(self, cmd):        # Used to serial Tx
        if (type(cmd) == str):
            self.ser.write(cmd.encode())
        elif (type(cmd) == bytes):
            self.ser.write(cmd)


    def send_duty_vals(self):
        self.curr_joy[0] = self.left_motor
        self.curr_joy[1] = self.right_motor

        right_low = (self.right_motor & 0b0000_0000_1111_1111)
        right_high = (self.right_motor >> 8)
        left_low = (self.left_motor & 0b0000_0000_1111_1111)
        left_high = (self.left_motor >> 8)


        if (self.curr_joy != self.last_joy):

            if (self.left_motor == 2999 and self.right_motor == 2999):
                self.get_logger().info(f'In deadzone')
            else:
                self.get_logger().info(f'Joystick moving')
        
            self.send(b'2')     # send message_type 2
            time.sleep(0.05)
            
            self.send((left_high).to_bytes(1, byteorder="big"))
            time.sleep(0.05)

            self.send((left_low).to_bytes(1, byteorder="big"))
            time.sleep(0.05)

            self.send((right_high).to_bytes(1, byteorder="big"))
            time.sleep(0.05)

            self.send((right_low).to_bytes(1, byteorder="big"))
            time.sleep(0.05)

            ########### output to ROS terminal ####
            self.get_logger().info(f'Left = {self.left_motor}  {format(self.left_motor, "016b")}')
            self.get_logger().info(f'Right = {self.right_motor}  {format(self.right_motor, "016b")}')
            # self.get_logger().info(f'lh = {format(left_high, "08b")}')
            # self.get_logger().info(f'll = {format(left_low, "08b")}')
            # self.get_logger().info(f'rh = {format(right_high, "08b")}')
            # self.get_logger().info(f'rl = {format(right_low, "08b")}')
            ###############################################################

        self.last_joy[0] = self.left_motor
        self.last_joy[1] = self.right_motor



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
        self.left_motor = int( (self.left_motor * 1000) + 2999 )
        self.right_motor = int( (self.right_motor * 1000) + 2999 ) 



    def joy_callback(self, msg):
        ''' Callback function grabs some of the values being published by /joy topic and sends serially to Arduino. '''

        self.button_values = msg.buttons
        # self.get_logger().info(f'Subber received buttons = {self.button_values}')

        # self.get_logger().info(f'button values = {self.button_values}')
        self.button_values.pop(11)      # gets last 4 unused buttons out of the array so button values is a 8 element array
        self.button_values.pop(10)
        self.button_values.pop(9)
        self.button_values.pop(8)
        self.button_values = bitarray(self.button_values)      # converts array into a byte array
        # self.get_logger().info(f'after pops = {self.button_values}')

        self.axes_values = msg.axes
        # self.get_logger().info(f'Subber received axes = {self.axes_values}')



        ''' Motor control using the Joysticks '''
        # TODO pull out curr_value/last value comparison out here so we can avoid doing computation here if there is no val difference

        # If left joystick is outside of deadzone
        if (self.axes_values[0] > self._deadzone or self.axes_values[0] < -(self._deadzone) or self.axes_values[1] > self._deadzone or self.axes_values[1] < -(self._deadzone)):
            self.arcade_drive_math(self.axes_values[0], self.axes_values[1])    # calc left and right motor values
            self.send_duty_vals()

        # if left joystick is within deadzone
        else:       
            self.left_motor = 2999
            self.right_motor = 2999
            self.send_duty_vals()


        if (self.last_butt != self.button_values):
            self.send(b'1')     # send message_type 1
            time.sleep(0.05)

            self.send(bytes(self.button_values))       # converts bit array to byte and sends
            time.sleep(0.05)

            self.send(b'0')     # send filler bytes
            time.sleep(0.05)

            self.send(b'0')
            time.sleep(0.05)

            self.send(b'0')
            time.sleep(0.05)     
            
            # self.get_logger().info(f'Vals: {self.button_values}')
            # self.get_logger().info(f'Sent: {bytes(self.button_values)}')


        self.last_butt = self.button_values



def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = GamepadSubber()    # creates a node instance

    subber.left_motor = 2999
    subber.right_motor = 2999
    subber.send_duty_vals()

    rclpy.spin(subber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()
    self.ser.close()


if __name__ == '__main__':
    main()
