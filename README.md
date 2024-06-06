# NASA Lunabotics 2024 - Team ARES for SDSU

Using Ubuntu 20.04 LTS on Jetson & Ubuntu 22.04 LTS on laptop.  ROS2 foxy on Jetson & ROS2 humble on laptop.  Different ROS2 versions shouldn't affect their ability to work together over a network.  
Python packages needed for controls:  rclpy, pyserial, sensor_msgs, bitarray  
Python packages needed for modified Vulcan video stream:  cv2, pyrealsense2, flask, numpy  


## Rover telecoms set up:
Ensure laptop & Jetson are both connected to the ARES_WAP

`ros2 run joy joy_node` on laptop to detect Logitech controller

`minicom -D /dev/<arduino_USB_port> -b 500000` on Jetson to serial monitor the Arduino Mega

`python3 serial_joystick.py` on Jetson

<run vulcan video stream on Jetson command>

<connect to appropriate IP address to view video stream on laptop>


## Passwords:
| Device              | Username | Password        |                         |
| :-----------------: | :------: | :-------------: | :---------------------: |
| Laptop              | steui    | WARHAWK         |                         |
| Jetson              | stubert  | S2P@C619        |                         |
| WAP connection      | ARES_WAP | 72878781        |                         |
| WAP config settings | admin    | DoctorShaffar<3 | Website: tplinkwifi.net |

## On Arduino Mega hat:
J17 connects to PB6 (drivetrain R) & PB5(drivetrain L)  
J18 connects to DH3 (exc tilt) & PL4 (depo tilt)  
J19 connects to DH5 (exc height) & PH4 (exc chain)  

## Logitech controller buttons mapping:
| Control        | Use                              |
| -------------: | :------------------------------  |
| Left joystick  | Drivetrain controls              |
| Right joystick | Camera gimble controls           |
| B or X         | Excavation chain fwd or backward |
| Y or A         | Excavation frame move up or down |
| LB or LT       | Depo bin tilt out or in          |
| RB or RT       | Excavation frame tilt out or in  |
