from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  return LaunchDescription([
    Node(package='gscam',
         executable='gscam_node',
         name='gscam_node',
         parameters=[
            {'camera_info_url' : 'package://gscam/examples/uncalibrated_parameters.ini'},
            {'gscam_config' : 'v4l2src do-timestamp=true ! video/x-raw ! jpegenc ! multipartmux ! multipartdemux ! jpegparse'},
            {'camera_name' : 'default'},
            {'image_encoding' : 'jpeg'},
            {'use_gst_timestamps' : 'true'},
            {'sync_sink' : 'true'},
         ]
    )
  ])