#!/usr/bin/env python3

import rospy
import numpy as np

from roscopter_msgs.msg import Command

class LAWNMOWER():
    
    def __init__(self):
        self.command_pub = rospy.Publisher('high_level_command', Command, queue_size=10)

        altitude = -2

        self.cmd_msg = Command()
        self.cmd_msg.mode = Command.MODE_NPOS_EPOS_DPOS_YAW
        self.cmd_msg.cmd3 = altitude
        self.cmd_msg.cmd4 = 0

        corners = [[5,-2],[9,-2],[9,2],[5,2]]
        density = 0.4

        waypoints = self.generateWaypoints(corners,density)

        dx = 0.1
        dt = 0.12

        self.generateTrajectory(waypoints,dx,dt)

        while not rospy.is_shutdown():
            rospy.spin()

    def generateWaypoints(self,corners,density):
    # function that takes four corner waypoints and a search density,
    # and outputs a lawnmower path. Corners go clockwise from SW.
        waypoints = []
        i = 0
        bool = False
        numPoints = 2*np.ceil((corners[3][1] - corners[0][1])/density) + 2
        while i < numPoints-2:
            # Add waypoints until there are just 2 left (the last two corners)
            if (i % 2) == 1:
                # bool==true means we are between 2nd and 3rd wp
                bool = not bool
            # While the E position of the lawnmower path hasn't crossed
            # the E position of the right corners
            waypoints.append([corners[bool][0], corners[bool][1] + np.floor(i/2)*density])
            i += 1

        if bool:
            waypoints.append(corners[2])
            waypoints.append(corners[3])
        else:
            waypoints.append(corners[3])
            waypoints.append(corners[2])

        # TODO: Send to middle of square upon completion?
        return waypoints

    def generateTrajectory(self,waypoints,dx,dt):
    # Function that takes in waypoints and speed, and publishes a waypoint
    # command at every dt; assumes only N and E line directions

        numEdges = len(waypoints) - 1
        i = 0
        while i < numEdges:
            dN = waypoints[i+1][0] - waypoints[i][0]
            if abs(dN) > 0.01:
                edgeLength = dN
                north = True
            else:
                edgeLength = waypoints[i+1][1] - waypoints[i][1]
                north = False
            dir = np.sign(edgeLength)
            numPoints = np.ceil(abs(edgeLength)/dx)
            j = 0
            while j < numPoints:
                command = [waypoints[i][0] + north*dir*j*dx,waypoints[i][1] + (not north)*dir*j*dx]
                j += 1
                self.publish(command)
                if rospy.is_shutdown():
                    break
                rospy.sleep(dt)
            i += 1
        
        command = waypoints[i]
        self.publish(command)

    def publish(self,waypoint):
        self.cmd_msg.stamp = rospy.Time.now()
        self.cmd_msg.cmd1 = waypoint[0]
        self.cmd_msg.cmd2 = waypoint[1]
        self.command_pub.publish(self.cmd_msg)

if __name__ == '__main__':
    rospy.init_node('lawnmower', anonymous=True)
    try:
        lawnmower = LAWNMOWER()
    except:
        rospy.ROSInterruptException
    pass