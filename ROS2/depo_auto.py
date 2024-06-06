import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from bitarray import bitarray
import time


left_motor = 2999
right_motor = 2999


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


def move(l, r):
    global left_motor
    global right_motor
    left_motor = l
    right_motor = r
    send_duty_vals()
    print_vals()
    time.sleep(0.20)


def move_stop():
    global left_motor
    global right_motor
    left_motor = 2999
    right_motor = 2999
    print_vals()
    send_duty_vals()


def print_vals():
    print(f'L Left = {left_motor}')
    print(f'L Right = {right_motor}')


def send_duty_vals():
    right_low = (right_motor & 0b0000_0000_1111_1111)
    right_high = (right_motor >> 8)
    left_low = (left_motor & 0b0000_0000_1111_1111)
    left_high = (left_motor >> 8)

    send(b'2')     # send message_type 2
    time.sleep(0.05)
    
    send((left_high).to_bytes(1, byteorder="big"))
    time.sleep(0.05)

    send((left_low).to_bytes(1, byteorder="big"))
    time.sleep(0.05)

    send((right_high).to_bytes(1, byteorder="big"))
    time.sleep(0.05)

    send((right_low).to_bytes(1, byteorder="big"))
    time.sleep(0.05)

    print_vals()


def tilt_fwd():
    send(b'1')     # send message_type 1
    time.sleep(0.05)

    send(bytes(b'00001000'))
    time.sleep(0.05)

    filler_bytes()


def tilt_back():
    send(b'1')     # send message_type 1
    time.sleep(0.05)

    send(bytes(b'00000010'))

    time.sleep(0.05)
    filler_bytes()



def main(args=None):

    # ser = serial.Serial('/dev/ttyACM0', 500000, bytesize=8, timeout=2)      # serial to Arduino Mega

    move_stop()

    # Move back
    print("\nMoving back")
    timer = 0
    while (left_motor > 2369 and right_motor > 2369) and (timer < 2):
        l = left_motor - 30
        r = right_motor - 30
        move(l, r)
        time.sleep(0.2)
        timer += 0.2

    print("\nStopping")
    move_stop()
    time.sleep(0.75)

    # Tilt fwd
    print("\nTilting depo bin fwd")
    tilt_fwd()
    time.sleep(5)
    tilt_fwd()
    time.sleep(5)
    tilt_fwd()
    time.sleep(5)
    
    # Tilt back
    print("\nTilting depo bin back")
    tilt_back()
    time.sleep(5)
    tilt_back()
    time.sleep(5)
    tilt_back()
    time.sleep(5)
    


if __name__ == '__main__':
    main()
