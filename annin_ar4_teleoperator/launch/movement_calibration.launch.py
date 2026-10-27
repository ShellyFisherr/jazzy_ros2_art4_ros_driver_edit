from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    movement_calibration = Node(
        package="annin_ar4_teleoperator",
        executable="movement_calibration",
        name="movement_calibration",
        output="screen",
    )

    return LaunchDescription([movement_calibration])
