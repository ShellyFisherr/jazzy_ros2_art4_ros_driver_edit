import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def load_yaml(package_name, file_path):
    package_share = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_share, file_path)

    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)

def generate_launch_description():

  document = load_yaml("tracker_udp_bridge", 
                       os.path.join("config", "PortSettings.yaml"),
                      )["PortSettings"]["ros__parameters"]

  PortParameters = {"udp_ip": document["udp_ip"],
                "udp_port": document["port"],
                }

  return LaunchDescription([
    

    Node(
      package='tracker_udp_bridge',
      executable='tracker_udp_bridge',
      name='tracker_udp_bridge',
      output='screen',
      parameters=[PortParameters],
      
    )
  ])