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


		
	
		

class robot_controller():

	def __init__(self):
		self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size = 2)
		self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback)
    
	def compute(self):
		angle = 0.0
		speed = 0.0
		twist = Twist()
		twist.linear.x = speed
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
		
		(rows,cols,channels) = cv_image.shape
		if cols > 60 and rows > 60 :
			cv2.circle(cv_image, (50,50), 10, 255)
		
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)
		
		self.compute()
		
		
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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
  

	ic = robot_controller()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
