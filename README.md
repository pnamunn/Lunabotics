# NASA Lunabotics 2024 - Team ARES for SDSU

Using Ubuntu 20.04 LTS on Jetson & laptop.  ROS2 foxy on Jetson & ROS2 humble on laptop.
Python packages needed for controls:  rclpy, pyserial, sensor_msgs, bitarray
Python packages needed for Vulcan video stream:  cv2, pyrealsense2, flask, numpy


## Rover telecoms set up:
`ros2 run joy joy_node` on laptop

`ros2 run serial_gamepad` on Jetson

`ros2 run current sensor pubber` on Jetson

`ros2 run current sensor subber` on laptop

run vulcan video stream on Jetson

connect to appropriate IP address to view video stream on laptop

Optional:  Use `minicom -D /dev/<arduino_USB_port> -b 500000` on the Jetson to serial monitor the Arduino Mega