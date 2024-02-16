
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

        self.deadzone = 0.2
        
        self.axes_values_last = [0 for i in range(6)]   #idk if initializing like this matters/is correct
        self.butt_values_last = [0 for i in range(6)]   


    def send(self, cmd):        # Used to serial write ASCII cmds
        if (type(cmd) == str):
            self.ser.write(cmd.encode())
        elif (type(cmd) == bytes):
            self.ser.write(cmd)


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

    def send_motor_data(self, left, right):      
        self.left_high = left >> 8                  #splits flag values into 2 bytes
        self.left_low = left & (255)
        self.right_high = right >> 8
        self.right_low = right & (255)

        self.send(4)                                # i think this is the command for joystick control
        self.send(self.left_high)                   # we send the flag
        self.send(self.left_low)
        self.send(self.right_high)
        self.send(self.right_low)

        self.send('l')                              # to fill the buffer on embedded side. Prob want to have it be end byte instead
        self.send('o')
        self.send('l')




    def joy_callback(self, msg):
        ''' Callback function grabs some of the values being published by /joy topic, converts
        them to ASCII values, and sends serially to Arduino. '''

        self.button_values = msg.buttons
        self.get_logger().info(f'Subber received buttons = {self.button_values}')

        self.axes_values = msg.axes
        self.get_logger().info(f'Subber received axes = {self.axes_values}')
        

        ''' Motor control using the Joysticks '''
        if (self.axes_values == self.axes_values_last):     #checks for changes in controller input
        # if left joystick is out of deadzone, math the flags
            if (self.axes_values[0] > self.deadzone or self.axes_values[0] < -(self.deadzone)
                or self.axes_values[1] > self.deadzone or self.axes_values[1] < -(self.deadzone)):   # if Left Joystick is in neutral
                
                self.joystick_math(self.axes_values[0], self.axes_values[1])   
                self.send_motor_data(self.left,self.right)


            else:       # if left joystick is inside of deadzone
                self.send_motor_data(3000, 3000)

        if (self.button_values != self.butt_values_last):
            for p in range(8):                                  # convert button array into 1 byte
                self.butt_bin += self.button_values[p] << p
            
            self.send(2)
            self.send(self.butt_bin)

            self.send('l')                              # same reason as the other one
            self.send('o')
            self.send('l')
            self.send('o')
            self.send('l')
            self.send('o')



        self.butt_values_last = self.button_values          # update the old array
        self.axes_values_last = self.axes_values
        self.butt_bin = 0

        ''' Motor control using the Dpad '''
        # if (self.axes_values[1] == -0.0 and self.axes_values[0] == -0.0):      # No presses on Dpad occurring
        #     if (self.button_values[4] == 1):      # LB pressed
        #         self.send('1')        # retract
        #     elif (self.button_values[6] == 1):    # LT pressed
        #         self.send('3')      # extend
        #     else:
        #         self.send('m')        # send stop by sending an unused ASCII value

        # elif (self.axes_values[1] == 1.0):      # Dpad up is pressed
        #     self.send('w')        # move forward
        # elif (self.axes_values[1] == -1.0):      # Dpad down is pressed
        #      self.send('s')        # move backward
        # elif (self.axes_values[0] == 1.0):      # Dpad left is pressed
        #     self.send('a')        # skid steer left
        # elif (self.axes_values[0] == -1.0):      # Dpad right is pressed
        #     self.send('d')        # skid steer right


        # ''' Depo bin controls '''
        # if (self.button_values[X] == 1):    # RB pressed
        #     self.send('j')      # tilt up
        # elif (self.button_values[X] == 1):    # RT pressed
        #     self.send('l')      # tilt down




def main(args=None):
    
    rclpy.init(args=args)       # inits rclpy library

    subber = GamepadSubber()    # creates a node instance

    rclpy.spin(subber)       # spins node (endlessly loops) until the user kills the node program (with Ctrl+C)

    subber.destroy_node()   
    rclpy.shutdown()
    self.ser.close()


if __name__ == '__main__':
    main()
