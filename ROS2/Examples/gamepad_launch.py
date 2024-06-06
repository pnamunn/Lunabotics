''' Starts a joy node & a serial_gamepad node'''

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', 
             executable='joy_node', 
             namespace='gamepad_control',       # groups the node into this system
             name='joy_node',                   # names the node
        ),
        Node(package='gamepad_pkg', 
             executable='serial_gamepad',
             namespace='gamepad_control',   
             name='serial_gamepad_node',
        )
    ])
