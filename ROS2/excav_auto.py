import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from bitarray import bitarray
import time


def send(cmd):        # Used to serial Tx
    # if (type(cmd) == str):
    #     self.ser.write(cmd.encode())
    # elif (type(cmd) == bytes):
    #     self.ser.write(cmd)
    pass


def filler_bytes():
    send(b'0')     # send 3 filler bytes
    time.sleep(0.05)

    send(b'0')
    time.sleep(0.05)

    send(b'0')
    time.sleep(0.05)

# Excavate AND go down
def excavate_down():
    send(b'1')     # send message_type 1
    time.sleep(0.05)

    send(bytes(b'01100000'))
    time.sleep(0.05)

    filler_bytes()

# Bring height back up
def height_up():
    send(b'1')     # send message_type 1
    time.sleep(0.05)

    send(bytes(b'00010000'))
    time.sleep(0.05)

    filler_bytes()

def main(args=None):

    # ser = serial.Serial('/dev/ttyACM0', 500000, bytesize=8, timeout=2)      # serial to Arduino Mega

    # Height will go down and dig at same time
    print("\nExcavating and going down")
    excavate_down()
    time.sleep(5)
    excavate_down()
    time.sleep(5)   
    
    # Height goes back up
    print("\nGoing back up")
    height_up()
    time.sleep(5)
    height_up()
    time.sleep(5)
    

if __name__ == '__main__':
    main()
