#!/usr/bin/env python3

import rospy
import numpy as np

from roscopter_msgs.msg import Command
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

class AUTONOMY():
    
    def __init__(self):
        self.command_pub = rospy.Publisher('high_level_command', Command, queue_size=10)
        self.gradient_pub = rospy.Publisher('gradient_ascent', Float64MultiArray, queue_size=10)

        self.laser_sub = rospy.Subscriber('laser_strength', Float64, self.laserCallback, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback, queue_size=10)

        self.cmd_msg = Command()
        self.odom_msg = Odometry()
        self.laser_msg = Float64()
        self.I = 0

        # ---------- Things to Change: -----------
        self.altitude = -1.75
        self.maxspeed = 0.6 # m/s
        self.spacing = 0.6
        self.I_min = 5 #.07 #TODO Reset to 0.07 for night flying
        # self.guidance = 1                               # Hold Position
        self.guidance = 2                               # Gradient Ascent
        self.find_delay = 5                             # Seconds to pause to see if signal is good
        self.delaying = False
        self.delay_start = 0

        self.pos_hold = [0, 0, self.altitude, 0]

        self.initialize = True
        self.searching = False


        # Gradient Ascent Params
        self.ga_init = True
        self.ga_min_distance = 0.01
        self.alpha = 0.05                               # For low pass filter
        self.beta = 0.0001                              # For decaying filter
        self.gamma = 0.4                                # GA proportional gain

        while not rospy.is_shutdown():
            rospy.spin()

    def autonomy(self):
        self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DPOS_YAW
        self.cmd_msg.cmd3 = self.altitude
        self.cmd_msg.cmd4 = 0

        if self.initialize:                             # First, go to the commanded position
            self.cmd_msg.cmd1 = self.pos_hold[0]
            self.cmd_msg.cmd2 = self.pos_hold[1]
            self.initialize = False
        elif self.I < self.I_min and not self.delaying:      # If there is no laser signal, search
            self.ga_init = True
            self.search()
        else:                                           # If there is a laser signal:
            time = rospy.Time.now()
            if self.searching:
                print('Found It')                          # If you were searching, stop and hold
                self.searching   = False
                self.delaying    = True
                self.delay_start = time.to_sec()
                self.pos_hold[0] = self.current_pos[0]
                self.pos_hold[1] = self.current_pos[1]
                self.I_max = self.I
            elif self.I > self.I_max:                   # If you drifted to a better place, hold there
                self.pos_hold[0] = self.current_pos[0]
                self.pos_hold[1] = self.current_pos[1]
                self.I_max = self.I
            if time.to_sec() - self.delay_start > self.find_delay:
                self.delaying = False
            if self.delaying:
                self.cmd_msg.cmd1 = self.pos_hold[0]
                self.cmd_msg.cmd2 = self.pos_hold[1]
            elif self.guidance == 1:                      # Position Hold
                self.cmd_msg.cmd1 = self.pos_hold[0]
                self.cmd_msg.cmd2 = self.pos_hold[1]
            elif self.guidance == 2:                    # Gradient Ascent
                self.gradientAscentOutputOnly()
            elif self.guidance == 3:                    # EKF Tracking - TODO
                self.cmd_msg.cmd1 = self.pos_hold[0]
                self.cmd_msg.cmd2 = self.pos_hold[1]

        self.publish()                                  # Publish the command

    def search(self):
        t_time = rospy.Time.now()
        if not self.searching:
            print('Searching')
            self.searching = True
            self.radius = self.spacing
            self.t_start = t_time.to_sec()
        omega = self.maxspeed/self.radius
        t = t_time.to_sec() - self.t_start
        if omega*t < 2*3.1415:
            self.cmd_msg.cmd1 = self.radius*np.sin(omega*t) + self.pos_hold[0]
            self.cmd_msg.cmd2 = self.radius*np.cos(omega*t) + self.pos_hold[1]
        else:
            self.t_start = t_time.to_sec()
            self.radius = self.radius + self.spacing

    def gradientAscent(self):
        x = np.array(self.current_pos[0:2])
        if self.ga_init:
            self.x_prev = x
            self.I_prev = self.I
            self.gradient_prev = np.array([0.001, 0.001])
            self.ga_init = False
        if np.linalg.norm(x - self.x_prev) > self.ga_min_distance:
            prefiltered_gradient = (self.I - self.I_prev)/(x - self.x_prev)
            gradient = self.alpha*prefiltered_gradient + (1-self.alpha)*self.gradient_prev
            self.x_prev = x
            self.I_prev = self.I
            self.gradient_prev = gradient
        else:
            gradient = (1 - self.beta)*self.gradient_prev
            self.gradient_prev = gradient
        vel_command = self.gamma*gradient
        # saturate
        vel_sat = self.maxspeed*vel_command/np.linalg.norm(vel_command)

        self.cmd_msg.mode = Command.MODE_NVEL_EVEL_DPOS_YAWRATE
        self.cmd_msg.cmd1 = vel_sat[0]
        self.cmd_msg.cmd2 = vel_sat[1]
        self.cmd_msg.cmd3 = self.altitude
        self.cmd_msg.cmd4 = -2*self.current_pos[3]

    def gradientAscentOutputOnly(self):
        x = np.array(self.current_pos[0:2])
        if self.ga_init:
            self.x_prev = x
            self.I_prev = self.I
            self.gradient_prev = np.array([0.001, 0.001])
            self.ga_init = False
        if np.linalg.norm(x - self.x_prev) > self.ga_min_distance:
            prefiltered_gradient = (self.I - self.I_prev)/(x - self.x_prev)
            gradient = self.alpha*prefiltered_gradient + (1-self.alpha)*self.gradient_prev
            self.x_prev = x
            self.I_prev = self.I
            self.gradient_prev = gradient
        else:
            gradient = (1 - self.beta)*self.gradient_prev
            self.gradient_prev = gradient
        vel_command = self.gamma*gradient
        # saturate
        vel_sat = self.maxspeed*vel_command/np.linalg.norm(vel_command)

        # Keep the command mode the same
        self.cmd_msg.cmd1 = self.pos_hold[0]
        self.cmd_msg.cmd2 = self.pos_hold[1]

        gradient_msg = Float64MultiArray()
        gradient_msg.data = vel_sat
        self.gradient_pub.publish(gradient_msg)


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

        self.autonomy()
    
    def laserCallback(self, laser_msg):
        self.I = laser_msg.data

    def publish(self):
        self.cmd_msg.stamp = rospy.Time.now()
        self.command_pub.publish(self.cmd_msg)

if __name__ == '__main__':
    rospy.init_node('autonomy', anonymous=True)
    try:
        autonomy = AUTONOMY()
    except:
        rospy.ROSInterruptException
    pass
