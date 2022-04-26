#!/usr/bin/env python3

import rospy
import math

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class LASERMEASUREMENT():

    def __init__(self):
        rospy.Subscriber('camera/odom/sample', Odometry, self.positionCallback, queue_size=10)
        self.pub = rospy.Publisher('expected_measurement', Float64, queue_size=10)
        
        while not rospy.is_shutdown():
            rospy.spin()

    def positionCallback(self,msg):
        x = msg.pose.pose.position.y + 0.57
        z = 1
        I_expected = Float64()
        I_expected.data = self.laserMeasurement(x,z)

        self.pub.publish(I_expected)

    def laserMeasurement(self,x,z):
        theta = 12*3.1415926535/180
        n = 8
        I = 18.17/(z+8.13)**2*math.exp(-2*(x/(z*math.sin(theta/2)))**n)

        return I


if __name__ == '__main__':
    rospy.init_node('laser_measurement', anonymous=True)
    try:
        laser_measurement = LASERMEASUREMENT()
    except:
        rospy.ROSInterruptException
    pass