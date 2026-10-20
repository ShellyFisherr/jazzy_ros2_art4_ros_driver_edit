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
    package_name = "annin_ar4_teleoperator"

    scaling_document = load_yaml(package_name, os.path.join("config", "translation_scaling.yaml"),
    )
    calibration_document = load_yaml(package_name, os.path.join("config", "calibration.yaml"),
    )

    scaling_parameters = scaling_document["translation_scaling"]["ros__parameters"]
    calibration = calibration_document["calibration"]

    rotation = calibration["rotation"]
    translation = calibration["translation"]

    target_frame = scaling_parameters["target_frame"]
    if target_frame != calibration["target_frame"]:
        raise RuntimeError(
            "target_frame in translation_scaling.yaml must match "
            "target_frame in calibration.yaml"
        )

    parameters = dict(scaling_parameters)
    parameters.update({
        "tracker_frame": calibration["source_frame"],
        "tracker_to_target_rotation_xyzw": [
            float(rotation["x"]),
            float(rotation["y"]),
            float(rotation["z"]),
            float(rotation["w"]),
        ],
        "tracker_to_target_translation_xyz": [
            float(translation["x"]),
            float(translation["y"]),
            float(translation["z"]),
        ],
        "result_file": os.path.join(get_package_share_directory(package_name), "config", "translation_scaling_result.yaml",
        ),
    })

    node = Node(
        package=package_name,
        executable="translation_scaling",
        name="translation_scaling",
        output="screen",
        parameters=[parameters],
    )

    return LaunchDescription([node])
