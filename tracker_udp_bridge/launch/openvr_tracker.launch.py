import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  

  return LaunchDescription([
    Node(
      package='tracker_udp_bridge',
      executable='openvr_tracker',
      name='openvr_tracker',
      output='screen',
    )
  ])