#!/usr/bin/env python2

import rospy
import json
import csv, os
import logging
from std_msgs.msg import String
import rospkg
import time

# initialization
if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/move/cmd_str', String, queue_size=10)
        rospy.init_node('circle')
        rate = rospy.Rate(1) # 1hz
        now = time.time()
        future = now + 5
        while time.time() < future and not rospy.is_shutdown():
            rospy.loginfo("Turning Left.")
            pub.publish("left")

        pub.publish("stop")
    except Exception as e:
        logging.exception("An error occurred")