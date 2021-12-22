#!/usr/bin/env python2

import rospy
import json
import os
import logging
from std_msgs.msg import String

CONTROLLER = os.environ['MOTOR_CONTROLLER'].lower()
if CONTROLLER == 'adafruit':
    from Adafruit_MotorHAT import Adafruit_MotorHAT
elif CONTROLLER == 'qwiic':
    import qwiic_scmd
else:
    raise ImportError


class CircleRobot:
    motor_left_ID = 1
    motor_right_ID = 2
    max_pwm = 255

    def __init__(self):
        self.setup_motor_controller()
        self.last_known_command_time = rospy.Time.now()

    def setup_motor_controller(self):
        rospy.loginfo("Starting with controller %s", CONTROLLER)
        if CONTROLLER == 'adafruit':
            # setup motor controller
            self.motor_driver = Adafruit_MotorHAT(i2c_bus=1)

            self.motor_left = self.motor_driver.getMotor(CircleRobot.motor_left_ID)
            self.motor_right = self.motor_driver.getMotor(CircleRobot.motor_right_ID)

            # stop the motors as precaution
            self.all_stop()
        elif CONTROLLER == 'qwiic':
            self.motor_driver = qwiic_scmd.QwiicScmd()
            self.motor_driver.disable()

    def circle(self, throttle_pct=0):
        speed = int(throttle_pct * self.max_pwm)

        rospy.loginfo("speed: %d", speed)

        self.motor_driver.set_drive(0, 0, speed)
        self.motor_driver.set_drive(1, 1, speed)
        self.motor_driver.enable()

    def main(self):
        self.circle(0.25)
        rospy.sleep(10)
        self.motor_driver.disable()
        rospy.spin()


# initialization
if __name__ == '__main__':
    try:
        rospy.init_node('circle')
        jetbot = CircleRobot()
        jetbot.main()
    except Exception as e:
        logging.exception("An error occurred")
