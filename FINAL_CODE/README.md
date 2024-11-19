### In competition, the following code was used to control the rover:

- serial_joystick.py : 
	- Input: ROS2 datatypes describing the current state of the gamepad's joysticks & buttons
	- Output: Encoded serial bytes of the gamepad's state changes
- ARES_Embedded.c : 
	- Input: Encoded serial bytes of the gamepad's state changes
	- Output: 16-bit PWM signals for motors & actuators
- vulcan_camera_modified.py :
	- *Input:* Frames from the Intel Realsense camera
	- *Output:* A Flask app, accessible by other devices on the same network, that shows a grayscale video stream  
  
  
### Before competition, the following code was finished and successfully tested individually, but the team did not have enough time to intergrate them with the rest of the C code on one microcontroller:

- current_sensors.c
	- Input: 
	- Output: Prints current readings from 2 sensors, as floating points with 3 sig figs
- loadcell_sensor.c
	- Input: 4 strain gauges in Wheatstone configuration, fed through the HX711 ADC/PGA
	- Output: Prints detected weight 
- proximity_sensors.c
	- Input: 
	- Output: Prints distance from detected obstacle & blocks drivetrain movement if obstacle becomes too close