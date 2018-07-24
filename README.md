# Autonomous RC car ROS system
This repos contains all the code developped for the autonomous RC car challenge of Continental, Renault, Airbus and Thalès at the FAB14 event as part of a hackaton. Please note that the solution is far from being optimized and has been devlopped in 20h (3D printing parts included). A specific focus in the ContiTeam has been to create a very high level architecture, based on ROS which enables further devlopements and portability on other platforms. 

## The circuit :
Information about the circuit can be found [here](https://github.com/kolergy/Fab14-DroneEvent).

The track is also designed for a drone race, which was held above the cars heads.
  ![Screenshot](https://github.com/kolergy/Fab14-DroneEvent/blob/master/Circuit_V0.4a.png)

## The Hackathon kit :

Every teams were provided with a kit including :
   - a [1/10 brushed drift car kit](https://www.banggood.com/Sinohobby-MINI-Q-Slash-TR-Q7BL-128-Carbon-Fiber-Racing-Brushless-RC-Car-p-1217972.html?rmmds=search)
   - a raspberry Pi 3
   - a raspi camera v2
   - a PCA 9685 I2C/PWM converter
   
 to which we added : 
   - a 3A 12v-5v regulator
   - a 3A 12v-3.3V regulator
     
## The mechanics

Everything stacked as fast as possible to get something running :

![Screenshot](https://github.com/Luczia/autonomousRCcar/blob/master/doc/TheBeast.jpg)

The stock car being waaay too fast  (10m/s at full throttle) for a non-optimized raspberry Pi vision control, we designed a special gear to reduce by 1/5th the speed and have a better accuracy in speed control (and be able to run a 0.5m/s at minimum throttle). 3D printed files are available in the [mechanics](https://github.com/Luczia/autonomousRCcar/tree/master/mechanics) folder.


## Software Arhitecture

Everything is developped for ROS (kinetic) environment, the system uses line tracking (color and contour) solutions to steer the wheels and adapt propulsion. The wheel are controlled py a PPM ServoMotor and the propulsion is driven by a RC car 8 A ESC. The car was designed to follow a 20m long track with a white line and borders.
COnsidering the very low devlopment time (1 week or 10 spare hours), the algorithm focuses on line tracking.

The Raspberry was using a ubuntu 16.04 image with ros-desktop-full install. It generates its own Wi-fi network on which a ground station can connect through a ROS-NETWORK for monitoring and debugging.

The system is based on 3 nodes in 3 packages : 


- **pwmDriver.py** in **driver_mot** which translates twist_messages into PWM instruction on the PCA 9685 through I2C.      
- **raspicam_node** from UbiquityRobotics https://github.com/UbiquityRobotics/raspicam_node      
- **convert.py** in **autnomous_vision** which registers to rapi_cam_node image and uses OpenCV instructions to extract the line 
      The node publishes the different stages of image processing as ros_images in the ros_workspace for debugging from the ground station and it generated a dynamic reconfigure server to allow online thresholding of the contour and color binarization.
      
  ![Screenshot](https://github.com/Luczia/autonomousRCcar/blob/master/doc/Screenshot%20from%202018-07-24%2022-58-37.png)
      
      
 The algorithm has 4 steps:
   - Filter white (and red) colors and output a binarized image as a white blob
   - Extract contours, sort the biggest (which should be the white line with correct cropping) and then enhance the contour before binarizing it. 
   - The algorithm finally extract the white blobs left when overlapping binary images from the color extraction and the contour extraction.
   - It finally computes the center of mass of the blob which is supposed to be the line.
   
 Then the cars steers the wheel through a simple proportional coefficient according to the x position of the mass center of the extracted blob .
 
 ![Screenshot](https://github.com/Luczia/autonomousRCcar/blob/master/doc/IMG_20180715_233203.jpg)
 
 

 
 To run, please launch :
```
 roslaunch driver_mot driver.launch 
```
 
 
 [Video of first trials](https://github.com/Luczia/autonomousRCcar/blob/master/doc/VID_20180717_143601.mp4) (with wrong coefficient)
 

