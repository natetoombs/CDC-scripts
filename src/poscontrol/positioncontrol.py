#!/usr/bin/env python3

import rospy
import numpy as np

from roscopter_msgs.msg import Command
from nav_msgs.msg import Odometry
from keyboard.msg import Keyboard

class POSITIONCONTROL():
    
    def __init__(self):
        self.command_pub = rospy.Publisher('high_level_command', Command, queue_size=10)

        self.key_sub = rospy.Subscriber('keyboard/binary', Keyboard, self.keyCallback, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback, queue_size=10)

        x_hold = 0
        y_hold = 0
        z_hold = -2
        yaw_hold = 0

        self.hold_pos = [x_hold,y_hold,z_hold,yaw_hold]
        self.current_pos = self.hold_pos

        self.cmd_msg = Command()
        self.odom_msg = Odometry()

        self.xyspeed = 0.75 # m/s
        self.zspeed = -0.5 # m/s

        self.z_pos = -2

        # self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DPOS_YAW
        # self.cmd_msg.cmd3 = altitude
        # self.cmd_msg.cmd4 = 0


        while not rospy.is_shutdown():
            rospy.spin()

    def odomCallback(self, odom_msg):
        # get current position
        n = odom_msg.pose.pose.position.x
        e = odom_msg.pose.pose.position.y
        d = odom_msg.pose.pose.position.z

        # orientation in quaternion form
        qw = odom_msg.pose.pose.orientation.w
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z

        # yaw from quaternion
        psi = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

        self.current_pos = [n, e, d, psi]

    def keyCallback(self, key_msg):
        # NWU to NED SHOULD BE ALREADY DONE
        if key_msg.K_w == 1:
            self.cmd_msg.mode = Command.MODE_NVEL_EVEL_DPOS_YAWRATE
            self.cmd_msg.cmd1 = self.xyspeed #*np.cos(self.current_pos[3])
            self.cmd_msg.cmd2 = 0 #self.xyspeed*np.sin(self.current_pos[3])
            self.cmd_msg.cmd3 = self.z_pos
            self.cmd_msg.cmd4 = -0.5*self.current_pos[3]
            self.hold_pos = self.current_pos
        elif key_msg.K_s == 1:
            self.cmd_msg.mode = Command.MODE_NVEL_EVEL_DPOS_YAWRATE
            self.cmd_msg.cmd1 = -self.xyspeed #*np.cos(self.current_pos[3])
            self.cmd_msg.cmd2 = 0 #-self.xyspeed*np.sin(self.current_pos[3])
            self.cmd_msg.cmd3 = self.z_pos
            self.cmd_msg.cmd4 = -0.5*self.current_pos[3]
            self.hold_pos = self.current_pos
        elif key_msg.K_d == 1:
            self.cmd_msg.mode = Command.MODE_NVEL_EVEL_DPOS_YAWRATE
            self.cmd_msg.cmd1 = 0 #self.xyspeed*np.sin(self.current_pos[3])
            self.cmd_msg.cmd2 = self.xyspeed #*np.cos(self.current_pos[3])
            self.cmd_msg.cmd3 = self.z_pos
            self.cmd_msg.cmd4 = -0.5*self.current_pos[3]
            self.hold_pos = self.current_pos
        elif key_msg.K_a == 1:
            self.cmd_msg.mode = Command.MODE_NVEL_EVEL_DPOS_YAWRATE
            self.cmd_msg.cmd1 = 0 #-self.xyspeed*np.sin(self.current_pos[3])
            self.cmd_msg.cmd2 = -self.xyspeed #*np.cos(self.current_pos[3])
            self.cmd_msg.cmd3 = self.z_pos
            self.cmd_msg.cmd4 = -0.5*self.current_pos[3]
            self.hold_pos = self.current_pos
        elif key_msg.K_e == 1:
            self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DVEL_YAW
            self.cmd_msg.cmd1 = self.hold_pos[0]
            self.cmd_msg.cmd2 = self.hold_pos[1]
            self.cmd_msg.cmd3 = self.zspeed
            self.cmd_msg.cmd4 = 0 #self.hold_pos[3]
            self.z_pos        = self.current_pos[2]
        elif key_msg.K_q == 1:
            self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DVEL_YAW
            self.cmd_msg.cmd1 = self.hold_pos[0]
            self.cmd_msg.cmd2 = self.hold_pos[1]
            self.cmd_msg.cmd3 = -self.zspeed
            self.cmd_msg.cmd4 = 0 #self.hold_pos[3]
            self.z_pos        = self.current_pos[2]
        else:
            self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DPOS_YAW
            self.cmd_msg.cmd1 = self.hold_pos[0]
            self.cmd_msg.cmd2 = self.hold_pos[1]
            self.cmd_msg.cmd3 = self.z_pos
            self.cmd_msg.cmd4 = 0 #self.hold_pos[3]
        
        self.publish()


    def publish(self):
        self.cmd_msg.stamp = rospy.Time.now()
        self.command_pub.publish(self.cmd_msg)

if __name__ == '__main__':
    rospy.init_node('positioncontrol', anonymous=True)
    try:
        positioncontrol = POSITIONCONTROL()
    except:
        rospy.ROSInterruptException
    pass
