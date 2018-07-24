# autonomousRCcar
All the code developped for the autonomousRCcar challenge of Continental, Renault, Airbus and Thalès at the FAB14 event


Every teams were provided with a kit including :
   - a 1/10 th brushed drift car kit https://www.banggood.com/Sinohobby-MINI-Q-Slash-TR-Q7BL-128-Carbon-Fiber-Racing-Brushless-RC-Car-p-1217972.html?rmmds=search
   - a respberry Pi 3
   - a raspi camera v2
   - a PCA 9685 I2C/PWMconverter
   
 to which we added : 
     -a 3A 12v-5v regulator and 12v-3.3V regulator



Everything is developped for ROS environment, the system uses line tracking (color and contour) solutions to steer the wheels and adapt propulsion. The wheel are controlled py a PPM ServoMotor and the propulsion is driven by a RC car 8 A ESC.



