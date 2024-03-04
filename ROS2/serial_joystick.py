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
import time


# Node that subs to /joy pubber
class GamepadSubber(Node):

    def __init__(self):     # Constructor

        super().__init__('gamepad_subber_node')  

        ''' Creates a subber node of msg_type=Joy, topic_name=joy, callback_function=joy_callback() '''
        self.subber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info("GamepadSubber(Node) instance created")

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=5)      # serial to Arduino Mega 

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



    def send_duty_vals(self):
        self.curr_joy[0] = self.left_motor
        self.curr_joy[1] = self.right_motor

        right_low = str(self.right_motor & 0b0000_0000_1111_1111)
        right_high = str(self.right_motor >> 8)
        left_low = str(self.left_motor & 0b0000_0000_1111_1111)
        left_high = str(self.left_motor >> 8)


        if (self.curr_joy != self.last_joy):

            self.send('2')     # send message_type
            time.sleep(0.4)

            if (self.ser.read() == ']'):    # if Arduino sent an ack
                self.send(left_high)    
                self.get_logger().info('\n\n ARDUINO ACK RECEIVED \n\n')   
                pass
            else:
                self.send('?')      # tell Arduino to expect msg_type next
                self.get_logger().info('\n\n ARDUINO SENT A NACK WOMP WOMP \n\n')   
                return

            # self.send(left_high)            # send left_motor high
            # time.sleep(0.4)


            if (self.ser.read() == ']'):    # if Arduino sent an ack
                self.send(left_low)    
                self.get_logger().info('\n\n ARDUINO ACK RECEIVED \n\n')   
                pass
            else:
                self.send('?')      # tell Arduino to expect msg_type next
                self.get_logger().info('\n\n ARDUINO SENT A NACK WOMP WOMP \n\n')   
                return

            # self.send(left_low)   # send left_motor low
            # time.sleep(0.4)


            if (self.ser.read() == ']'):    # if Arduino sent an ack
                self.send(left_low)    
                self.get_logger().info('\n\n ARDUINO ACK RECEIVED \n\n')   
                pass
            else:
                self.send('?')      # tell Arduino to expect msg_type next
                self.get_logger().info('\n\n ARDUINO SENT A NACK WOMP WOMP \n\n')   
                return
            
            # self.send(right_high)            # send right_motor high
            # time.sleep(0.4)

            if (self.ser.read() == ']'):    # if Arduino sent an ack
                self.send(left_low)    
                self.get_logger().info('\n\n ARDUINO ACK RECEIVED \n\n')   
                pass
            else:
                self.send('?')      # tell Arduino to expect msg_type next
                self.get_logger().info('\n\n ARDUINO SENT A NACK WOMP WOMP \n\n')   
                return

            # self.send(right_low)   # send right_motor low
            # time.sleep(0.4)

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
        self.left_motor = int( (self.left_motor * 1000) + 3000 )
        self.right_motor = int( (self.right_motor * 1000) + 3000 ) 

        self.get_logger().info(f'When X = {x}   Y = {y}')   
        self.get_logger().info(f'Left Motor = {self.left_motor}')
        self.get_logger().info(f'Right Motor = {self.right_motor}')

        self.get_logger().info(f'L = {bin(self.left_motor)}')
        self.get_logger().info(f'R = {bin(self.right_motor)}')
    



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
            self.send_duty_vals()

        # if left joystick is within deadzone
        else:       
            self.get_logger().info(f'In deadzone')

            self.left_motor = 3000
            self.right_motor = 3000

            self.send_duty_vals()



        
   



def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = GamepadSubber()    # creates a node instance

    rclpy.spin(subber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()
    self.ser.close()


if __name__ == '__main__':
    main()
