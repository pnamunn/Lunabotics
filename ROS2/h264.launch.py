from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  return LaunchDescription([
    Node(package='gscam',
         executable='gscam_node',
         name='gscam_node',
         output='screen',
         parameters=[
            {'camera_info_url' : 'package://gscam/examples/uncalibrated_parameters.ini'},
            {'gscam_config' : 'v4l2src do-timestamp=true ! video/x-raw,framerate=30/1 ! v4l2h264enc \
             ! h264parse'},
# Pipeline is: 
# v4l2 source with sync timestamps on  ->  outputs x-raw video  ->  takes all color out of the x-raw video
#  ->  x-raw video is encoded into jpegs  ->  sequential jpegs are muxed into a buffer  ->  jpegs are demuxed  ->  outputs jpegs for sink
            {'camera_name' : 'default'},
            {'image_encoding' : 'jpeg'},
            {'use_gst_timestamps' : True},
            {'sync_sink' : True}
         ]
    )
  ])