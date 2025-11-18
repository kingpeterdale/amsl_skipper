from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

cam_params = os.path.join(get_package_share_directory('amsl_skipper'),'config','cam_param.yaml')

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='usb_cam',
      executable='usb_cam_node_exe',
      name='usb_cam',
      arguments=['--ros-args', '--params-file', cam_params] #, '--log-level', 'debug']
    ),

    Node(
      package='aruco_markers',
      executable='aruco_markers',
      name='aruco_markers',
      arguments=[
        '--ros-args',
        '-p', 'marker_size:=0.18',
        '-p', 'image_topic:=image_raw',
        '-p', 'camera_info_topic:=camera_info',
        '-p', 'dictionary:=DICT_4X4_50',
        '-p', 'camera_frame:=camera'
      ]
    )
 ])
