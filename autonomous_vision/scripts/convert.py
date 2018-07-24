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
	
	mode_robot = True
	
	def draw_lines(self, img, lines, color=(255, 0, 0), thickness=2):
	    for line in lines:
	        for x1,y1,x2,y2 in line:
	            cv2.line(img, (x1, y1), (x2, y2), color, thickness)
	
	
	def callbackCfgSrv(self, config, level):
	    rospy.loginfo("""Reconfigure Request: {int_sensibility},{int_sensibility2},{int_sensibility3},{int_sensibility4},{double_lspeed},{double_1},{double_2},{bool_param}""".format(**config))
	    self.sensibility = config["int_sensibility"]
	    self.canny = config["int_sensibility2"]
	    self.canny2 = config["int_sensibility3"]
	    # ~ print("Config:")
	    # ~ print(config["int_sensibility"])
	    #print(self.sensibility)
	    self.lspeed = config["double_lspeed"]
	    self.bool_param = config["bool_param"]
	    return config

	def callbackCfg(config):
		rospy.loginfo("Config set to {int_sensibility}, {double_lspeed}, {double_param}, {bool_param}".format(**config))		
		# ~ self.sensibility = config["int_sensibility"]
        # ~ self.lspeed = config["double_lspeed"]
        print("CallBack")
		

	def __init__(self):
		#Create Dynamic reconfigure Server
		if self.mode_robot == True : 
			self.cfg_srv = Server(CfgrobotConfig, self.callbackCfgSrv)	
		
		#Create publishers
		self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size = 1)
		self.image_reg_white = rospy.Publisher("image_setting_white",Image, queue_size = 1)
		self.image_reg_red = rospy.Publisher("image_setting_red",Image, queue_size = 1)
		self.image_reg_canny = rospy.Publisher("image_setting_canny",Image, queue_size = 1)
		self.image_reg_contour = rospy.Publisher("image_setting_contour",Image, queue_size = 1)
		self.image_reg_final = rospy.Publisher("image_setting_final",Image, queue_size = 1)
		
		
		self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		
		#Create subscribers
		self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback)
		#self.cfg_client = dynamic_reconfigure.client.Client("autonomous_vision", timeout=30, config_callback=self.callbackCfg)
		
		self.bridge = CvBridge()
		
		self.lspeed = 0
		
		self.filter_values = np.array([])
		
		
    
	def sendCommand(self, angle, lspeed):
		twist = Twist()
		twist.linear.x = lspeed
		twist.angular.z = angle
		self.twist_publisher.publish(twist)
		
	def median_slide(self, new_value):
		number_filter_value = 10
		
		self.filter_values = np.append(new_value, self.filter_values)
		
		if np.size(self.filter_values) > number_filter_value:
			self.filter_values = np.resize(self.filter_values, number_filter_value)

		return np.median(self.filter_values)
			
	def median_reset():
		self.filter_values =  np.array([])

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
		crop_img = cv_image[2*height/4:height][1:width]
		height, width, channels = crop_img.shape
		
		
		#If we want to activate the contour detection solution
		if self.bool_param == True :
		
	###Canny
			edges = cv2.Canny(crop_img,self.canny,self.canny2)
			#add bottom line
			cv2.line(edges,(0,height-5),(width,height-5),(255,0,0),5)
			
		###Add Hough Transform
			lines = cv2.HoughLinesP(edges, 0.8, np.pi/180, 25, np.array([]), 50, 200)
			line_img = np.zeros(edges.shape, dtype=np.uint8)
			self.draw_lines(line_img, lines)
						
			
		####Methode Find Contour
			# find contours in the edged image, keep only the largest ones, and initialize our screen contour
			_, contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			cnts = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
			screenCnt = None
			ctr_image_org = cv2.drawContours(crop_img, cnts[0], -1, (0, 255, 0), 3)
			#Generating Contour Binary Image
			bgn = np.zeros((height,width,3), np.uint8)
			ctr_image = cv2.drawContours(bgn, cnts[0], -1, (255, 255, 255), 40)
			ctr_image_bin = cv2.threshold(ctr_image, 254, 255, cv2.THRESH_BINARY)[1]
			ctr_image_bin = ctr_image_bin[:,:,2]
			
			
		#### Method Fill Holes 
			# Copy the thresholded image.
			im_floodfill = ctr_image.copy()		 
			# Mask used to flood filling.
			# Notice the size needs to be 2 pixels than the image.
			h, w = ctr_image.shape[:2]
			# Mask used to flood filling.
			# Notice the size needs to be 2 pixels than the image.
			mask = np.zeros((h+2, w+2), np.uint8)		 
			cv2.floodFill(im_floodfill, mask, (0,0), 255);		 
			# Invert floodfilled image
			im_floodfill_inv = cv2.bitwise_not(im_floodfill)		 
			# Combine the two images to get the foreground.
			im_out = ctr_image | im_floodfill_inv
			# ~ cv2.imshow("Image Infill", np.hstack([ctr_image,im_floodfill_inv,im_out]))
		

	###Methode BGR		
		# ~ bgr = [40, 158, 16]
		
		# ~ thresh = 40
		# ~ lower = np.array([bgr[0] - thresh, bgr[1] - thresh, bgr[2] - thresh], dtype = "uint8")
		# ~ upper = np.array([bgr[0] + thresh, bgr[1] + thresh, bgr[2] + thresh], dtype = "uint8")
		
		# ~ mask = cv2.inRange(crop_img, lower, upper)
		# ~ extraction = cv2.bitwise_and(crop_img, crop_img, mask = mask)
		

	# Methode HSV			
		brightHSV = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
		
		#White
		sensitivity = self.sensibility			
		minHSV = np.array([0,0,255-sensitivity])
		maxHSV = np.array([255,sensitivity,255])		 
		maskHSVwhite = cv2.inRange(brightHSV, minHSV, maxHSV)
		#resultHSVwhite = cv2.bitwise_and(crop_img, crop_img, mask = maskHSV)
		#cv2.imshow("maskHSV", np.hstack([maskHSV]))
		#resultHSVwhiteBin = cv2.cvtColor(resultHSVwhite, cv2.COLOR_HSV2BGR)
		#resultHSVwhiteBin = cv2.cvtColor(resultHSVwhiteBin, cv2.COLOR_BGR2GRAY)
		#resultHSVwhiteBin = cv2.threshold(resultHSVwhite, 127, 255, cv2.THRESH_BINARY)[1]	
		#Blue and Red
		lower_blue = np.array([110,50,50])
		upper_blue = np.array([130,255,255])
		lower_red = np.array([169, 100, 100], dtype=np.uint8)
		upper_red = np.array([189, 255, 255], dtype=np.uint8)			
		maskHSVred = cv2.inRange(brightHSV, lower_red, upper_red)	
		#resultHSVred = cv2.bitwise_and(crop_img,crop_img, mask= mask)
		#resultHSVredBin = cv2.threshold(resultHSVred, 127, 255, cv2.THRESH_BINARY)[1]
		#Display Result
	
			
		
		#Finalize the image before processing
		if self.bool_param == True :
			resultExtract = cv2.bitwise_and(maskHSVwhite, ctr_image_bin)
		else :
			# ~ resultExtract = cv2.bitwise_and(maskHSVwhite, maskHSVed)
			resultExtract = maskHSVwhite
		# ~ print(type(maskHSVwhite[0]))
		# ~ print(maskHSVwhite[0][0])
		# ~ print(type(ctr_image_bin[0]))
		# ~ print(ctr_image_bin[0][0])
		#resultExtractBin = cv2.inRange(resultExtract, 0 ,255)
	
		#Extract barycenter			
		m = cv2.moments(resultExtract, True)
		try:
		  x, y = m['m10']/m['m00'], m['m01']/m['m00']
		except ZeroDivisionError:
		  x, y = height/2, width/2


		x = self.median_slide(x)
		cv2.circle(crop_img,(int(x), int(y)), 2,(255,255,0),10)

		# ~ cv2.imshow("Image Final", np.hstack([resultExtract]))
		# ~ cv2.imshow("Image FinalBin", np.hstack([resultExtractBin]))
		
		# ~ np.vstack([  np.hstack([crop_img,extraction,resultHSV])   ,   np.hstack([edges,line_img,line_img])  ])
		
		
		if self.mode_robot == False : 
			cv2.imshow("Image Bary", np.hstack([resultExtract]))
			cv2.imshow("Image Ctr Detection", np.hstack([crop_img,ctr_image,ctr_image_org]))
			cv2.imshow("Image white&Red Detection", np.hstack([maskHSVred,maskHSVwhite]))
			cv2.imshow("Image Hough", np.hstack([edges, line_img])) 
		
		
		cv2.waitKey(1)
		
		kp = 0.6
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
		if self.mode_robot == True : 
			self.sendCommand(cmd_angular,self.lspeed)
			# ~ print("Send Command")
		
		#Publish images on ROS workspace
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)
		try:
			self.image_reg_white.publish(self.bridge.cv2_to_imgmsg(maskHSVwhite, "mono8"))
		except CvBridgeError as e:
			print(e)
		try:
			self.image_reg_red.publish(self.bridge.cv2_to_imgmsg(maskHSVred, "mono8"))
		except CvBridgeError as e:
			print(e)
			
		if self.bool_param == True :
			try:
				self.image_reg_canny.publish(self.bridge.cv2_to_imgmsg(edges, "mono8"))
			except CvBridgeError as e:
				print(e)
			try:
				self.image_reg_contour.publish(self.bridge.cv2_to_imgmsg(ctr_image_bin, "mono8"))
			except CvBridgeError as e:
				print(e)
			try:
				self.image_reg_final.publish(self.bridge.cv2_to_imgmsg(resultExtract, "mono8"))
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
