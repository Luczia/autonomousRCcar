#!/usr/bin/env python

from __future__ import division
from time import sleep
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# Import the PCA9685 module.
import Adafruit_PCA9685

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Alternatively specify a different address and/or bus:
pwm = Adafruit_PCA9685.PCA9685(address=0x40,busnum=1)

# Configure min and max servo pulse lengths
servo_rotation_id = 0
servo_rotation_min = 160  # Min pulse length out of 4096
servo_rotation_max = 630  # Max pulse length out of 4096
servo_rotation_range = servo_rotation_max - servo_rotation_min

# Configure min and max servo pulse lengths
servo_translation_id = 1
servo_translation_min = 300  # Min pulse length out of 4096
servo_translation_max = 365  # Max pulse length out of 4096
servo_translation_range = servo_translation_max - servo_translation_min

pwm.set_all_pwm(0, 1)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
	

def SetAngle(angle):
	pwm.set_pwm(0, 0, angle)
	

def SetSpeed(speed):
	pwm.set_pwm(1, 0, speed)
	


def callbackTwist(data):
	linear_speed =  365 - (data.linear.x * 65)
	angular_position = (data.angular.z * (470)) + 160 + (470/2)
	
	# ~ rospy.loginfo(rospy.get_caller_id() + "LSpeed: %s - APosition : %s", linear_speed, angular_position)
	SetAngle(int(angular_position))
	SetSpeed(int(linear_speed))
	

def callbackSpeed(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard Int %s", data)
	SetAngle(data.data)
	

def callbackAngle(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard Int %s", data)
	SetAngle(data.data)
	

def listener():
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	

if __name__ == '__main__':
	rospy.init_node('DriverMoteur', anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, callbackTwist)
	listener()

