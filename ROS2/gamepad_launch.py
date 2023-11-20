''' Starts a joy node & a teleop_twist_joy node'''

import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='joy', executable='joy_node', name='joy_node',
                                arguments=['--ros-args', '--log-level', 'info']),
        launch_ros.actions.Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
                                arguments=['--ros-args', '--log-level', 'info'])
    ])