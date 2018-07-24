#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('autonomous_vision')
import sys
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# numpy and scipy
import numpy as np
import cv2

from dynamic_reconfigure.server import Server
from autonomous_vision.cfg import CfgrobotConfig

import dynamic_reconfigure.client



class robot_controller():
	
	
	def callbackCfgSrv(self, config, level):
	    rospy.loginfo("""Reconfigure Request: {int_sensibility}, {double_lspeed},{double_param}, {bool_param}""".format(**config))
	    self.sensibility = config["int_sensibility"]
	    # ~ print("Config:")
	    # ~ print(config["int_sensibility"])
	    #print(self.sensibility)
	    self.lspeed = config["double_lspeed"]
	    self.bparam = config["bool_param"]
	    return config

	def callbackCfg(config):
		rospy.loginfo("Config set to {int_sensibility}, {double_lspeed}, {double_param}, {bool_param}".format(**config))		
		# ~ self.sensibility = config["int_sensibility"]
        # ~ self.lspeed = config["double_lspeed"]
        print("CallBack")
		

	def __init__(self):
		#Create Dynamic reconfigure Server
		self.cfg_srv = Server(CfgrobotConfig, self.callbackCfgSrv)	
		
		#Create publishers
		self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size = 2)
		self.image_reg_pub = rospy.Publisher("image_reglage",Image, queue_size = 2)
		self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
		#Create subscribers
		self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback)
		#self.cfg_client = dynamic_reconfigure.client.Client("autonomous_vision", timeout=30, config_callback=self.callbackCfg)
		
		self.bridge = CvBridge()
		self.sensibility = 60
		self.lspeed = 0
    
	def sendCommand(self, angle, lspeed):
		twist = Twist()
		twist.linear.x = lspeed
		twist.angular.z = angle
		self.twist_publisher.publish(twist)

	def callback(self,data):
		#~ print ("received image of type: ", data.format)
		#~ try:
			#~ cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			#~ print ("image_recu")
		#~ except CvBridgeError as e:
			#~ print(e)
		
		np_arr = np.fromstring(data.data, np.uint8)
		#image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
		cv_image = cv2.flip(image_np, 0 )
		
		
		### Add red circle
		# ~ (rows,cols,channels) = cv_image.shape
		# ~ if cols > 60 and rows > 60 :
			# ~ cv2.circle(cv_image, (50,50), 10, 255)
		
		
		####generate visualization image
		# ~ cv2.imshow("Image window", cv_image)
		# ~ cv2.waitKey(3)
		
####Add the vision system code here#########		
######		
		height, width, channels = cv_image.shape
		crop_img = cv_image[225:height][1:width]

	###Methode BGR		
		bgr = [40, 158, 16]
		thresh = 40
		
		lower = np.array([bgr[0] - thresh, bgr[1] - thresh, bgr[2] - thresh], dtype = "uint8")
		upper = np.array([bgr[0] + thresh, bgr[1] + thresh, bgr[2] + thresh], dtype = "uint8")
		
		mask = cv2.inRange(crop_img, lower, upper)
		extraction = cv2.bitwise_and(crop_img, crop_img, mask = mask)
		

	# Methode HSV	
		brightHSV = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
		
		hsv = cv2.cvtColor( np.uint8([[bgr]] ), cv2.COLOR_BGR2HSV)[0][0]
 
		minHSV = np.array([hsv[0] - thresh, hsv[1] - thresh, hsv[2] - thresh])
		maxHSV = np.array([hsv[0] + thresh, hsv[1] + thresh, hsv[2] + thresh])
		sensitivity = self.sensibility
		# ~ print(sensitivity)		
		minHSV = np.array([0,0,255-sensitivity])
		maxHSV = np.array([255,sensitivity,255])
		 
		maskHSV = cv2.inRange(brightHSV, minHSV, maxHSV)
		resultHSV = cv2.bitwise_and(brightHSV, brightHSV, mask = maskHSV)

	#Extract barycenter	
		
		m = cv2.moments(maskHSV, False)
		try:
		  x, y = m['m10']/m['m00'], m['m01']/m['m00']
		except ZeroDivisionError:
		  x, y = height/2, width/2
		cv2.circle(resultHSV,(int(x), int(y)), 2,(0,255,0),3)
		
		cv2.imshow("Image window", np.hstack([crop_img,extraction,resultHSV]))
		cv2.waitKey(1)
		
		kp = -1
		cmd_angular = (x/width*2 - 1)*kp
		#print (cmd_angular)
		# ~ yaw = 1500 + (x - width/2) * 1.5
		# ~ print("center=" + str(width/2) + "point=" + str(x) + "yaw=" +  str(yaw))
		# ~ throttle = 1900
		
		# ~ if (yaw > 1900):
		  # ~ yaw = 1900
		# ~ elif (yaw < 1100):
		  # ~ yaw = 1100
##############End of vision system code here###############"	
		
		
		### Extrapolate the motor command
		self.sendCommand(cmd_angular,self.lspeed)
		
		
		#Publish images on ROS workspace
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)
		
		try:
			self.image_reg_pub.publish(self.bridge.cv2_to_imgmsg(resultHSV, "bgr8"))
		except CvBridgeError as e:
			print(e)
	
		  
		#### Create CompressedIamge ####
		# ~ msg = Image()
		# ~ msg.header.stamp = rospy.Time.now()
		# ~ msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
		
		
		# Publish new image
		# ~ self.image_pub.publish(msg)
		# ~ self.image_pub.publish(self.cv2_to_imgmsg(cv_image, encoding="rgb8"))


def main(args):
	rospy.loginfo("Initializing node Autonomous Vision")
	rospy.init_node('image_converter', anonymous=True)
	ic = robot_controller()
	
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
