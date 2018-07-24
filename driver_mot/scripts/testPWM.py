#!/usr/bin/env python

import RPi.GPIO as GPIO
from time import sleep
import rospy
from geometry_msgs.msg import Twist

pin = 12

def SetAngle(angle):
	duty = angle / 18 + 2
	GPIO.output(pin, True)
	pwm.ChangeDutyCycle(duty)
	sleep(1)
	GPIO.output(pin, False)
	pwm.ChangeDutyCycle(0)


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear.x)
	#SetAngle(data)
	StartAngle = 60
	StopAngle = 160
	for i in range(StartAngle, StopAngle+1, (StopAngle-StartAngle)/2):
		SetAngle(i)
		sleep(1)
	#SetAngle(80)
	#sleep(1)
	#SetAngle(90)
	#sleep(1)
	#SetAngle(110)
	#sleep(1)

def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('DriverMoteur', anonymous=True)

	rospy.Subscriber("commandemoteur", Twist, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(pin, GPIO.OUT)

	pwm=GPIO.PWM(pin, 50)
	pwm.start(0)
	listener()


