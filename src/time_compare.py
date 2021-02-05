#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped

def compareCallback(msg):
    now = rospy.get_time()
    time = msg.header.stamp
    # pub.publish(now-time)
    print(now-time.to_sec())

def main():
    rospy.init_node('time_compare', anonymous=True)

    rospy.Subscriber('ned_pose', PoseStamped, compareCallback, queue_size=10)
    # pub = rospy.Publisher('time_difference', float, queue_size=10)
    
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass