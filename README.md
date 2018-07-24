# autonomousRCcar
All the code developped for the autonomousRCcar challenge of Continental, Renault, Airbus and Thalès at the FAB14 event


Every teams were provided with a kit including :
   - a 1/10 th brushed drift car kit https://www.banggood.com/Sinohobby-MINI-Q-Slash-TR-Q7BL-128-Carbon-Fiber-Racing-Brushless-RC-Car-p-1217972.html?rmmds=search
   - a respberry Pi 3
   - a raspi camera v2
   - a PCA 9685 I2C/PWMconverter
   
 to which we added : 
     -a 3A 12v-5v regulator and 12v-3.3V regulator



Everything is developped for ROS environment, the system uses line tracking (color and contour) solutions to steer the wheels and adapt propulsion. The wheel are controlled py a PPM ServoMotor and the propulsion is driven by a RC car 8 A ESC. The car was designed to follow a 20m long track with a white line and borders.
COnsidering the very low devlopment time (1 week or 10 spare hours), the algorithm focuses on line tracking.
Information about the circuit can be found here : https://github.com/kolergy/Fab14-DroneEvent


The Raspberry was using a ubuntu 16.04 image with ros-desktop-full install. It generates its own Wi-fi network on which a ground station can connect through a ROS-NETWORK for monitoring and debugging.

The system is based on 3 nodes : 
      - pwmDriver.py which translates twist_messages into PWM instruction on the PCA 9685 through I2C.      
      - a raspicam_node from UbiquityRobotics https://github.com/UbiquityRobotics/raspicam_node      
      - autnomous_node whichr egister to camera image and uses OpenCV instruction to extract the line 
      
      
      
 The algorithm process color (filtering white and red) and binarize an image. Then it also extract contours, sort the biggest (which should be the white line with correct cropping) and then enhance the ocntour and binarize. The algorithm finally computes the center of mass of the white blob left whith overlapping binary from the color extraction and the contour extraction.
 Then the cars steers the wheel through a simple proportional coefficient according to the x position of the bary center of the extracted blob which is supposed to be the line.
 
 ![alt text](https://raw.githubusercontent.com/Luczia/autonomousRCcar/doc/IMG_20180715_233203.jpg)
 
 
 Please, note as well that the car being waaay to fast  (12m/s at full throttle) for araspbery Pi vision control, we designed a pecial gear to reduce 1/5 the speed and have a better accuracy in speed control (and be able to run a 0.5m/s). 3D printed files are available in the mechanics folder.

