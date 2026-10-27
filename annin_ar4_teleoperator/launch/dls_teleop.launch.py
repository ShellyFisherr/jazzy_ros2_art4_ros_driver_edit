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

    calibration = load_yaml("annin_ar4_teleoperator",
                            os.path.join("config", "translation_scaling_result.yaml"),
    )["translation_scaling"]

    calibration_parameters = {"translation_scale": float(calibration["scale"]),
                              "robot_near": [float(value) for value in calibration["robot_near"]],                          
    }

    node_settings = load_yaml("annin_ar4_teleoperator",
                              os.path.join("config", "DLS_controller.yaml"),
    )["DLS_Controller"]["ros__parameters"]

    node_settings_parameters = {"CONTROL_DT": float(node_settings["controller_frequency"]),
                                "DEBUG": node_settings["show_logs"],
                                "deadzone_velocity": float(node_settings["deadzone_velocity"]),
                                "deadzone_twist": float(node_settings["deadzone_twist"]),
                                "timeout_age": float(node_settings["timeout_age"]),
                                "deadzone_translation_error": float(node_settings["deadzone_translation_error"]),
                                "deadzone_orientation_error": float(node_settings["deadzone_orientation_error"]),
                                "position_gain": float(node_settings["position_gain"]),
                                "orientation_gain": float(node_settings["orientation_gain"]),
                                "DLS_damping": float(node_settings["DLS_damping"]),
                                "velocity_limit": node_settings["use_velocity_limit"],
                                "acceleration_limit": node_settings["use_acceleration_limit"],
                                "max_velocity": [float(value) for value in node_settings["max_velocity"]],
                                "max_acceleration": [float(value) for value in node_settings["max_acceleration"]],
                                "braking_delay": float(node_settings["braking_delay"]),
                                "stopping_margin": float(node_settings["stopping_margin"]),
                                "brake_damping_gain": float(node_settings["brake_damping_gain"]),
                                "velocity_weight": float(node_settings["velocity_weight"]),
                                "position_weight": float(node_settings["position_weight"]),
                                "position_priority_distance" : float(node_settings["position_priority_distance"]),
                                "position_priority_max_multiplier": float(node_settings["position_priority_max_multiplier"]),
                                "end_effector_name": node_settings["end_effector_name"],
    }

    frame_translation = load_yaml("annin_ar4_teleoperator",
                                            os.path.join("config", "calibration.yaml"),
    )["calibration"]

    frame_translation_parameters = {"target_frame": frame_translation["target_frame"],
    }

    dls_node = Node(
        package="annin_ar4_teleoperator",
        executable="dls_teleoperation_controller",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            calibration_parameters,
            node_settings_parameters,
            frame_translation_parameters,
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
