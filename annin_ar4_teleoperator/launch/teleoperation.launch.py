import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TELEOP_PACKAGE = "annin_ar4_teleoperator"

def load_yaml(package_name, relative_path):
  package_share = get_package_share_directory(package_name)
  yaml_path = os.path.join(package_share, relative_path)

  with open(yaml_path, "r", encoding="utf-8") as yaml_file:
    return yaml.safe_load(yaml_file)


def check_success(event, stage_name):
  if event.returncode !=0:
    raise RuntimeError(
      f"{stage_name} failed with exit code {event.returncode}."
    )

# This is run after frame_translator starts. It waits for the configured
# transform to become available, rather than only checking that the process ran.

def generate_launch_description():
  simulation_arg = DeclareLaunchArgument(
    "simulation",
    default_value="true",
    choices=["true", "false"],
    description="Use Gazebo simulation instead of the real robot.",
  )
  calibration_arg = DeclareLaunchArgument(
    "calibration",
    default_value="true",
    choices=["true", "false"],
    description="Run movement and translation calibration.",
  )
  calibrate_arm_arg = DeclareLaunchArgument(
    "calibrate_arm",
    default_value="true",
    choices=["true", "false"],
    description="Run the real robot's startup calibration.",
  )
  tracker_arg = DeclareLaunchArgument(
    "tracker",
    default_value="linux",
    choices=["linux", "windows"],
    description="Tracker source: Linux OpenVR or Windows OpenVR via UDP bridge."
  )

  simulation = LaunchConfiguration("simulation")
  calibration = LaunchConfiguration("calibration")
  calibrate_hand = LaunchConfiguration("calibrate_hand")
  tracker = LaunchConfiguration("tracker")

  tracker_linux = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare("tracker_udp_bridge"),
        "launch",
        "openvr_tracker.launch.py",
      ])
    ),
    condition=IfCondition(
      PythonExpression(["'", tracker, "' == 'linux'"])
    ),
  )

  tracker_windows = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare("tracker_udp_bridge"),
        "launch",
        "tracker_udp_bridge.launch.py",
      ])
    ),
    condition = IfCondition(
      PythonExpression(["'", tracker, "' == 'windows'"])
    ),
  )

  wait_for_tracker_pose = ExecuteProcess(
    cmd=["timeout",
         "120",
         "ros2",
         "topic",
         "echo",
         "--once",
         "/tracker/pose",
    ],
    output="screen",
  )

  movement_calibration = Node(
    package=TELEOP_PACKAGE,
    executable="movement_calibration",
    name="movement_calibration",
    output="screen",
    emulate_tty=True,
  )

  def after_movement_calibration(event, context):
    check_success(event, "Movement calibration")
    return [OpaqueFunction(function=start_translation_scaling)]

  movement_calibration_handler = RegisterEventHandler(
    OnProcessExit(
      target_action=movement_calibration,
      on_exit=after_movement_calibration,
    )
  )

  def start_translation_scaling(context):
    scaling = load_yaml(
      TELEOP_PACKAGE,
      os.path.join("config", "translation_scaling.yaml"),
    )["translation_scaling"]["ros__parameters"]

    calibration_data = load_yaml(
      TELEOP_PACKAGE,
      os.path.join("config", "calibration.yaml"),
    )["calibration"]

    if scaling["target_frame"] != calibration_data["target_frame"]:
      raise RuntimeError(
        "target_frame in translation_scaling.yaml must match "
        "target_frame in calibration.yaml."
      )

    rotation = calibration_data["rotation"]
    translation = calibration_data["translation"]

    parameters = dict(scaling)
    parameters.update({
      "tracker_frame": calibration_data["source_frame"],
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
      "result_file": os.path.join(
        get_package_share_directory(TELEOP_PACKAGE),
        "config",
        "translation_scaling_result.yaml",
      ),
    })

    scaling_node = Node(
      package=TELEOP_PACKAGE,
      executable="translation_scaling",
      name="translation_scaling",
      output="screen",
      emulate_tty=True,
      parameters=[parameters],
    )

    def after_translation_scaling(event, context):
      check_success(event, "Translation scaling calibration")
      return [OpaqueFunction(function=start_frame_translator)]

    scaling_handler = RegisterEventHandler(
      OnProcessExit(
        target_action=scaling_node,
        on_exit=after_translation_scaling,
      )
    )

    return [scaling_handler, scaling_node]

  def start_frame_translator(context):
    calibration_data = load_yaml(
      TELEOP_PACKAGE,
      os.path.join("config", "calibration.yaml"),
    )["calibration"]

    rotation = calibration_data["rotation"]
    translation = calibration_data["translation"]

    frame_translator = Node(
      package=TELEOP_PACKAGE,
      executable="frame_translator",
      name="frame_translator",
      output="screen",
      parameters=[{
        "target_frame": calibration_data["target_frame"],
        "source_frame": calibration_data["source_frame"],
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
      }],
    )

    wait_for_translated_pose = ExecuteProcess(
      cmd=[
        "timeout",
        "30",
        "ros2",
        "topic",
        "echo",
        "--once",
        "/position_world",
        "geometry_msgs/msg/PoseStamped",
      ],
      output="screen",
    )

    def after_translated_pose(event, context):
      check_success(event, "Frame transform readiness check")
      return [OpaqueFunction(function=start_robot_stack)]

    transform_handler = RegisterEventHandler(
      OnProcessExit(
        target_action=wait_for_translated_pose,
        on_exit=after_translated_pose,
      )
    )

    return [transform_handler, frame_translator, wait_for_translated_pose]

  def start_robot_stack(context):
    use_sim_time = simulation.perform(context).lower()
    is_simulation = use_sim_time == "true"

    if is_simulation:
      robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          PathJoinSubstitution([
            FindPackageShare("annin_ar4_gazebo"),
            "launch",
            "gazebo.launch.py",
          ])
        ),
        launch_arguments={
          "use_sim_time": "true",
        }.items(),
      )
    else:
      robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          PathJoinSubstitution([
            FindPackageShare("annin_ar4_driver"),
            "launch",
            "driver.launch.py",
          ])
        ),
        launch_arguments={
          "calibrate": "true" if calibrate_hand.perform(context).lower() == "true"
          else "false",
          "include_gripper": "false",
        }.items(),
      )

    wait_for_joint_states = ExecuteProcess(
      cmd=[
        "timeout",
        "300",
        "ros2",
        "topic",
        "echo",
        "--once",
        "/joint_states",
      ],
      output="screen",
    )

    moveit = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        PathJoinSubstitution([
          FindPackageShare("annin_ar4_moveit_config"),
          "launch",
          "moveit.launch.py",
        ])
      ),
      launch_arguments={
        "use_sim_time": "True",
        "include_gripper": "False",
      }.items(),
    )

    dls_teleop = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        PathJoinSubstitution([
          FindPackageShare(TELEOP_PACKAGE),
          "launch",
          "dls_teleop.launch.py",
        ])
      ),
      launch_arguments={
        "use_sim_time": use_sim_time,
      }.items(),
    )


    def after_joint_states(event, context):
      check_success(event, "Robot joint-state readiness check")
      return [moveit, dls_teleop]


    joint_state_handler = RegisterEventHandler(
      OnProcessExit(
        target_action=wait_for_joint_states,
        on_exit=after_joint_states,
      )
    )

    return [robot_stack, joint_state_handler, wait_for_joint_states]

  def after_tracker_pose(event, context):
    check_success(event, "Tracker pose readiness check")

    if calibration.perform(context).lower() == "true":
      return [movement_calibration_handler, movement_calibration]

    return [OpaqueFunction(function=start_frame_translator)]

  tracker_pose_handler = RegisterEventHandler(
    OnProcessExit(
      target_action=wait_for_tracker_pose,
      on_exit=after_tracker_pose,
    )
  )

  return LaunchDescription([
    simulation_arg,
    calibration_arg,
    calibrate_arm_arg,
    tracker_arg,
    tracker_linux,
    tracker_windows,
    tracker_pose_handler,
    wait_for_tracker_pose,
  ])
  
