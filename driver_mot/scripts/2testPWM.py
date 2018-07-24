#!/usr/bin/env python

import RPi.GPIO as GPIO
from time import sleep
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

pin = 12

def SetAngle(angle):
	duty = angle / 18 + 2
	GPIO.output(pin, True)
	pwm.ChangeDutyCycle(duty)
	#sleep(0.05)
	#GPIO.output(pin, False)
	#pwm.ChangeDutyCycle(0)


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear.x)
	#SetAngle(data)
	StartAngle = 60
	StopAngle = 160
	for i in range(StartAngle, StopAngle+1, (StopAngle-StartAngle)/2):
		SetAngle(i)
		sleep(1)

def callbackInt(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard Int %s", data)
	#print(data.split(':')[1])
	SetAngle(data.data)
	#sleep(1)
	

def listener():
	
	

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(pin, GPIO.OUT)

	rospy.init_node('DriverMoteur', anonymous=True)
	rospy.Subscriber("commandemoteur", Twist, callback)
	rospy.Subscriber("testInt", Int16, callbackInt)

	pwm=GPIO.PWM(pin, 50)
	pwm.start(0)
	listener()

