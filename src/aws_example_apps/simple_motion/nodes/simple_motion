#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Import of python modules.
import math # use of pi.

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan

# Constants.
FREQUENCY = 10 #Hz.
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s
LASER_ANGLE_FRONT = 0 # radians
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance.
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan'

class SimpleMotion():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
        """Constructor."""

        # Setting up publishers/subscribers.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        # Setting up subscriber.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        # Other variables.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

    def move_forward(self, distance):
        """Function to move_forward for a given distance."""
        # Rate at which to operate the while loop.
        rate = rospy.Rate(FREQUENCY)

        # Setting velocities. 
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        start_time = rospy.get_rostime()
        duration = rospy.Duration(distance/twist_msg.linear.x)

        # Loop.
        while not rospy.is_shutdown():
            # Check if traveled of given distance based on time.
            if rospy.get_rostime() - start_time >= duration:
                break

            # Publish message.
            if self._close_obstacle:
                self.stop()
            else:
                self._cmd_pub.publish(twist_msg)

            # Sleep to keep the set publishing frequency.
            rate.sleep()

        # Traveled the required distance, stop.
        self.stop()
        
    def rotate_in_place(self, rotation_angle):
        """
        Rotate in place the robot of rotation_angle (rad) based on fixed velocity.
        Assumption: Counterclockwise rotation.
        """
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_velocity
        
        duration = rotation_angle / twist_msg.angular.z
        start_time = rospy.get_rostime()
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            # Check if done
            if rospy.get_rostime() - start_time >= rospy.Duration(duration):
                break
                
            # Publish message.
            self._cmd_pub.publish(twist_msg)
            
            # Sleep to keep the set frequency.
            rate.sleep()

        # Rotated the required angle, stop.
        self.stop()

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.
        i = int((LASER_ANGLE_FRONT - msg.angle_min) / msg.angle_increment)
        if msg.ranges[i] <= MIN_THRESHOLD_DISTANCE:
            self._close_obstacle = True
        else:
            self._close_obstacle = False


def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("simple_motion")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the simple motion.
    simple_motion = SimpleMotion()

    # If interrupted, send a stop command.
    rospy.on_shutdown(simple_motion.stop)

    # Robot moves 1 m forward, rotate of 180 degrees, and moves 1 m forward.
    try:
        simple_motion.move_forward(1)
        simple_motion.rotate_in_place(math.pi)
        simple_motion.move_forward(1)
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
