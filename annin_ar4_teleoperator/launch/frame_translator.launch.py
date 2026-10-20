import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)

    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)
    
def generate_launch_description():
    document = load_yaml("annin_ar4_teleoperator", os.path.join("config", "calibration.yaml"))

    calibration = document["calibration"]
    translation = calibration["translation"]
    rotation = calibration["rotation"]

    translation_parameters = {
        "target_frame": calibration["target_frame"],
        "source_frame": calibration["source_frame"],
        "translation_xyz": [
            float(translation["x"]),
            float(translation["y"]),
            float(translation["z"]),
        ],
        "rotation_xyzw": [
            float(rotation["x"]),
            float(rotation["y"]),
            float(rotation["z"]),
            float(rotation["w"]),
        ],
    }

    frame_translator = Node(
        package="annin_ar4_teleoperator",
        executable="frame_translator",
        name="frame_translator",
        output="screen",
        parameters=[translation_parameters],
    )

    return LaunchDescription([frame_translator])