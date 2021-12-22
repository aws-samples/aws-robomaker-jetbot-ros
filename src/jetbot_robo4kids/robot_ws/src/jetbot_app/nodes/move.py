#!/usr/bin/env python2
import rospy
import time
import json

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)

# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)
	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)

def move_dir(val):
	if val == "left":
		set_speed(motor_left_ID,  -0.2)
		set_speed(motor_right_ID,  0.2) 
	elif val == "right":
		set_speed(motor_left_ID,   0.2)
		set_speed(motor_right_ID, -0.2) 
	elif val == "backward":
		set_speed(motor_left_ID,   0.2)
		set_speed(motor_right_ID,  0.2)
	elif val == "forward":
		set_speed(motor_left_ID,  -0.2)
		set_speed(motor_right_ID, -0.2)  
	elif val == "stop":
		all_stop()
	else:
		rospy.logerror('Direction not supported.')

# directional commands (degree, speed)
def on_cmd_dir(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)

# velocity, twist commands (Twist)
def on_cmd_vel(msg):
    x = msg.linear.x
    y = msg.angular.z/10

    if x>0 and y<0: #backward right
    	rospy.loginfo(rospy.get_caller_id() + ', backward right (left, right)=(%s,%s)' % ((abs(y)+0.1), (0.2+y+0.1)))
    	set_speed(motor_left_ID, (abs(y)+0.1))
    	set_speed(motor_right_ID, (0.2+y+0.1))
    elif x>0 and y>0: #backward left
    	rospy.loginfo(rospy.get_caller_id() + ', backward left (left, right)=(%s,%s)' % ((0.2-y+0.1), (y+0.1)))
    	set_speed(motor_left_ID, (0.2-y+0.1))
    	set_speed(motor_right_ID, (y+0.1))
    elif x<0 and y>0: #forward left
    	rospy.loginfo(rospy.get_caller_id() + ', forward left (left, right)=(%s,%s)' % ((-(0.2-y)-0.1), -(y+0.1)))
    	set_speed(motor_left_ID, (-(0.2-y)-0.1))
    	set_speed(motor_right_ID, -(y+0.1))
    elif x<0 and y<0: #forward right
    	rospy.loginfo(rospy.get_caller_id() + ', forward right (left, right)=(%s,%s)' % (y-0.1, (-(0.2+y)-0.1)))
    	set_speed(motor_left_ID, y-0.1)
    	set_speed(motor_right_ID, (-(0.2+y)-0.1))
    else:
    	all_stop()
    
# raw L/R motor commands (speed, speed)
def on_cmd_raw(msg):
    rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)
    move_data_recv = json.loads(msg.data)
    set_speed(motor_left_ID, float(move_data_recv['left']))
    set_speed(motor_right_ID, float(move_data_recv['right']))

# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)
	move_dir(msg.data.lower())

# initialization
if __name__ == '__main__':

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('move')

	rospy.Subscriber('~cmd_dir', String, on_cmd_dir)
	rospy.Subscriber('~cmd_vel', Twist, on_cmd_vel)
	rospy.Subscriber('~cmd_raw', String, on_cmd_raw)
	rospy.Subscriber('~cmd_str', String, on_cmd_str)

	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()
