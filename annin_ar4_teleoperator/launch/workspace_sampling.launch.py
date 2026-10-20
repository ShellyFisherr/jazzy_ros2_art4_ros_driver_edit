import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command,
                                  FindExecutable,
                                  LaunchConfiguration,
                                  PathJoinSubstitution)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)

    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)

def generate_launch_description():
    #-------------------------------------
    # Launch Arguments
    #-------------------------------------

    use_sim_time = LaunchConfiguration("use_sim_time")
    include_gripper = LaunchConfiguration("include_gripper")
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation time.",
        ),

        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for the AR4 TF tree.",
        ),

        DeclareLaunchArgument(
            "include_gripper",
            default_value="False",
            choices=["True", "False"],
            description="Include the gripper in the robot model.",
        ),

        DeclareLaunchArgument(
            "ar_model",
            default_value="mk5",
            choices=["mk1", "mk2", "mk3", "mk4", "mk5"],
            description="AR4 robot model.",
        ),
    ]

    #-------------------------------------
    # URDF files (description)
    #-------------------------------------

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_description"),
            "urdf",
            "ar.urdf.xacro",
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "include_gripper:=",
        include_gripper,
    ])

    robot_description = {
        "robot_description": robot_description_content
    }

    #-------------------------------------
    # SRDF files (semantics)
    #-------------------------------------

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"),
            "srdf",
            "ar.srdf.xacro",
        ]),
        " ",
        "name:=",
        ar_model_config,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "include_gripper:=",
        include_gripper,
    ])

    robot_description_semantic = {
        "robot_description_semantic":
            robot_description_semantic_content
    }

    #-------------------------------------
    # Kinematics
    #-------------------------------------

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    #-------------------------------------
    # Workspace sampler
    #-------------------------------------

    workspace_parameters = os.path.join(
        get_package_share_directory("annin_ar4_teleoperator"),
        "config",
        "workspace.yaml",
    )

    #-------------------------------------
    # Start sampler only
    #-------------------------------------

    workspace_analyzer_node = Node(
        package="annin_ar4_teleoperator",
        executable="workspace_sampling",
        name="workspace_sampling",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            workspace_parameters,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        declared_arguments + [workspace_analyzer_node]
    )