#!/usr/bin/env python3

import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from roscopter_msgs.msg import Command

class ROV2COPTER():

    def __init__(self):
        # Set up subscriber and publisher
        self.rov_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.rovCallback, queue_size=10)
        self.pose_command_pub = rospy.Publisher('high_level_command', Command, queue_size=10)

        # Initialize geo_msg
        self.copter_msg = Command()

        self.pose_filtered = np.array([0, 0, 0])
        self.alpha = 0.05

        self.altitude = -1.3; # NED altitude

        while not rospy.is_shutdown():
        # wait for new messages and call the callback when they arrive
            rospy.spin()

    def rovCallback(self, geo_msg):
        # Need to convert ENU to NED, but set altitude to be fixed; don't need orientation
        self.copter_msg.stamp = rospy.Time.now()

        pose = np.array([geo_msg.pose.position.y, geo_msg.pose.position.x, self.altitude])

        self.pose_filtered = np.multiply(self.pose_filtered,(1-self.alpha)) + np.multiply(pose,self.alpha)


        self.copter_msg.cmd1 = self.pose_filtered[0] + 1
        self.copter_msg.cmd2 = self.pose_filtered[1]
        self.copter_msg.cmd3 = self.altitude
        self.copter_msg.cmd4 = 0
        self.copter_msg.mode = Command.MODE_NPOS_EPOS_DPOS_YAW
        # self.copter_msg.pose.orientation.x = geo_msg.pose.pose.orientation.x
        # self.copter_msg.pose.orientation.y = geo_msg.pose.pose.orientation.y
        # self.copter_msg.pose.orientation.z = geo_msg.pose.pose.orientation.z
        # self.copter_msg.pose.orientation.w = geo_msg.pose.pose.orientation.w

        # Publish message
        self.pose_command_pub.publish(self.copter_msg)

if __name__ == '__main__':
    rospy.init_node('rov2copter', anonymous=True)
    try:
        rov2copter = ROV2COPTER()
    except:
        rospy.ROSInterruptException
    pass