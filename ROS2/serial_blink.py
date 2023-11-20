import serial
import time


ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
time.sleep(2)

for i in range(10):
    ser.write(b'H')   # send a byte
    time.sleep(0.5)        # wait 0.5 seconds
    ser.write(b'L')   # send a byte
    time.sleep(0.5)

ser.close()