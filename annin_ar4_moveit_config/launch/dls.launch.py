import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    include_gripper = LaunchConfiguration("include_gripper")
    ar_model = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("annin_ar4_description"),
                "urdf",
                "ar.urdf.xacro",
            ]),
            " ",
            "ar_model:=",
            ar_model,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "include_gripper:=",
            include_gripper,
        ])
    }

    robot_description_semantic = {
        "robot_description_semantic": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("annin_ar4_moveit_config"),
                "srdf",
                "ar.srdf.xacro",
            ]),
            " ",
            "name:=",
            ar_model,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "include_gripper:=",
            include_gripper,
        ])
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"),
            "config/joint_limits.yaml",
        ]),
        allow_substs=True,
    )

    dls_node = Node(
        package="annin_ar4_keyboard_controll",
        executable="dls_reduced_controller",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("include_gripper", default_value="False"),
        DeclareLaunchArgument("ar_model", default_value="mk5"),
        dls_node,
    ])
