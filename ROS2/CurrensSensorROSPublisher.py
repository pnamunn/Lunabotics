import serial
import rclpy
from rclpy.node import Node
#import time
import struct
from std_msgs.msg import String 
import array
#from diagnostic_msgs import KeyValue.msg

#Service node will recieve an error message from the client
#will print out thier is a drop in current 
#will wait three seconds and then send an aknoledgement

class CurrentSensorPublisher(Node):
    def __init__(self):
        super().__init__('CurrentPublisher')
        #create a publisher node of type= KeyValue, topic= current, callback= current_callback, 
        self.publisher_ = self.create_publisher(String, 'current', 50)
        
        self.get_logger().info("Pubber node has been created")
        
        self.ser = serial.Serial('/dev/ttyACM0', 115200, bytesize=8, timeout = 1) #TODO find out of this is correct port

        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self): #only publish if a problem arrises
        #TODO fix Serial
        msg = String()
        data = 'LeftBack'                               #name of wheel with problem
        #request.value = 0                           
        #volts = 0.1
        volts = self.ser.read_until() 
        check = str(volts)
        i = 0
        while check[i] != '\\':
            i += 1
        number = check[2:i]
        #self.get_logger().info(number)

        testNum = float(number)
        print(testNum)
       # check = int.from_bytes(volts, "big")
       # arr = array.array('f')
     #  arr.frombytes(volts)
        #float_number = struct.unpack('f', volts[0-4])
        self.get_logger().info(volts)
     #  arr = int(volts)
        #self.get_logger().info(check)
        
        
        if testNum > 2: #if greater then 2 volts
            msg.data = 'Problem with wheel "' + data + '", current sesnor is reading "'+ str(volts) + '"volts.'
            self.publisher_.publish(msg)
            self.get_logger().info(msg.data)


        #time.wait(2)

def main(args=None):
    rclpy.init(args=args)           #get rclpy library
    pubber = CurrentSensorPublisher()      #create a node


    rclpy.spin(pubber)            #run until it is killed


    pubber.destroy_node()         #free node
    rclpy.shutdown()               #stop RoS


if __name__ == '__main__':
    main()
