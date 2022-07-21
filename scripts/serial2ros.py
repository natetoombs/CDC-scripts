#!/usr/bin/env python3

import rospy
import serial 
import argparse
from std_msgs.msg import Float64

class SERIAL2ROS():

    def __init__(self):
        self.serial_pub = rospy.Publisher('laser_strength', Float64, queue_size=10)
        
        ap = argparse.ArgumentParser()
        ap.add_argument("-p","--port",required = False, help = "Enter Port Name", default='/dev/ttyUSB0')
        ap.add_argument("-b","--baudrate",required = False, help = "Enter Baudrate", default=9600)
        args = vars(ap.parse_args())
        
        PORT = args['port']
        RATE = args['baudrate']

        self.attempts = 0
        while self.attempts < 10:
            self.print_serial(PORT,RATE) 

    def print_serial(self,port,rate):
        self.attempts = self.attempts + 11
        try:
            serial_port = serial.Serial(port,rate)
            print(f"The Port name is {serial_port.name}")
            while not rospy.is_shutdown():
                line = serial_port.readline()
                line = line.strip()
                line = line.decode("utf-8")
                if line != '':
                    line = float(line)
                    self.serial_pub.publish(line)
        except:
            print("SERIAL2ROS ERROR")
            print("check port")
            self.attempts = self.attempts - 10

if __name__ == '__main__':
    rospy.init_node('serial2ros', anonymous=True)
    try:
        serial2ros = SERIAL2ROS()
    except:
        rospy.ROSInterruptException
    pass
