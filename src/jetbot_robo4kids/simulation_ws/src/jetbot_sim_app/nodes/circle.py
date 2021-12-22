#!/usr/bin/env python

import rospy
import rospkg
import csv, os
from geometry_msgs.msg import Twist



class Circle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('jetbot_diff_controller/cmd_vel', Twist, queue_size=1)

    def circle_forever(self):
        self.twist = Twist()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.twist.angular.z = 0.1
            self._cmd_pub.publish(self.twist)
            rospy.loginfo("Circling robot: %s", self.twist)
            r.sleep()


def main():
    rospy.init_node('circle')
    try:
        circle = Circle()
        circle.circle_forever()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
