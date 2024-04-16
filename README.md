# NASA Lunabotics 2024 - Team ARES for SDSU

Using Ubuntu 20.04 LTS on Jetson & laptop.  ROS2 foxy on Jetson & ROS2 humble on laptop.
Python packages needed for controls:  rclpy, pyserial, sensor_msgs, bitarray
Python packages needed for Vulcan video stream:  cv2, pyrealsense2, flask, numpy


## Rover telecoms set up:
`ros2 run joy joy_node` on laptop

`minicom -D /dev/<arduino_USB_port> -b 500000` on Jetson to serial monitor the Arduino Mega

`ros2 run serial_joystick` on Jetson

`ros2 run current sensor pubber` on Jetson

`ros2 run current sensor subber` on laptop

run vulcan video stream on Jetson

connect to appropriate IP address to view video stream on laptop



## On Arduino Mega hat:
J17 connects to PB6 (drivetrain R) & PB5(drivetrain L)
J18 connects to DH3 (exc tilt) & PL4 (depo tilt)
J19 connects to DH5 (exc height) & PH4 (exc chain)