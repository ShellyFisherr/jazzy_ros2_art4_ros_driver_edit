import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  udp_ip = LaunchConfiguration('udp_ip')
  udp_port = LaunchConfiguration('udp_port')

  return LaunchDescription([
    DeclareLaunchArgument('udp_ip', default_value='0.0.0.0'),
    DeclareLaunchArgument('udp_port', default_value='5005'),

    Node(
      package='tracker_udp_bridge',
      executable='tracker_udp_bridge',
      name='tracker_udp_bridge',
      output='screen',
      parameters=[
        {'udp_ip': udp_ip},
        {'udp_port': udp_port},
        {'use_sim_time': False}
      ]
    )
  ])