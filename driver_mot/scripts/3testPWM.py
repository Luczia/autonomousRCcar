#!/usr/bin/env python

import RPi.GPIO as GPIO
from time import sleep
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16


pinSpeed = 5
pinAngle = 3

def SetAngle(angle):
	duty = angle
	GPIO.output(pinAngle, True)
	pwmAngle.ChangeDutyCycle(duty)
	
def DesactivateServo():
	pwmAngle.start(0)
	print("Killing Direction")

def SetSpeed(speed):
	duty = speed
	GPIO.output(pinSpeed, True)
	pwmSpeed.ChangeDutyCycle(duty)
	



def callbackTwist(data):
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear.x)
	linear_speed = ((-data.linear.x*5.0) + 7)
	angular_speed = (-data.angular.z*5.0) + 7
	if (data.angular.z > -0.1 and data.angular.z < 0.1):
		angular_speed = 0
		DesactivateServo()
	rospy.loginfo(rospy.get_caller_id() + "LSpeed: %s - ASpeed : %s", linear_speed, angular_speed)
	SetAngle(angular_speed)
	SetSpeed(linear_speed)
	

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
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(pinAngle, GPIO.OUT)
	GPIO.setup(pinSpeed, GPIO.OUT)

	rospy.init_node('DriverMoteur', anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, callbackTwist)
	#rospy.Subscriber("speed", Int16, callbackSpeed)
	#rospy.Subscriber("angle", Int16, callbackAngle)

	pwmSpeed=GPIO.PWM(pinSpeed, 50)
	pwmAngle=GPIO.PWM(pinAngle, 50)
	pwmSpeed.start(0)
	pwmAngle.start(0)
	listener()

