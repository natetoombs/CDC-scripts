#!/usr/bin/env python3

import rospy
import serial 
import argparse
from std_msgs.msg import Float64

class SERIAL2ROS():

    def __init__(self):
        self.serial_pub = rospy.Publisher('laser_strength', Float64, queue_size=10)
        
        ap = argparse.ArgumentParser()
        ap.add_argument("-p","--port",required = False, help = "Enter Port Name", default='dev\ttyUSB0')
        ap.add_argument("-b","--baudrate",required = False, help = "Enter Baudrate", default=9600)
        args = vars(ap.parse_args())
        
        PORT = args['port']
        RATE = args['baudrate']
        self.print_serial(PORT,RATE) 

    def print_serial(self,port,rate):
        try:
            serial_port = serial.Serial(port,rate)
            print(f"The Port name is {serial_port.name}")
            while not rospy.is_shutdown():
                lines = serial_port.readline()
                lines = lines.strip()
                line = float(lines.decode("utf-8"))
                self.serial_pub(line)
        except:
            print("SERIAL2ROS ERROR")
            print("check port")
            exit() 

if __name__ == '__main__':
    rospy.init_node('serial2ros', anonymous=True)
    try:
        serial2ros = SERIAL2ROS()
    except:
        rospy.ROSInterruptException
    pass