from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare("annin_ar4_gazebo"),
        "launch",
        "gazebo.launch.py"
      ])
    ),
    launch_arguments={
      "use_sim_time": "true",
    }.items(),
  )

  wait_for_joint_states = ExecuteProcess(
    cmd=["ros2", "topic", "echo", "--once", "/joint_states"],
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

  frame_translator = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare("annin_ar4_teleoperator"),
        "launch",
        "frame_translator.launch.py",
      ])
    )
  )

  dls_teleop = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare("annin_ar4_teleoperator"),
        "launch",
        "dls_teleop.launch.py",
      ])
    )
  )

  def start_teleoperation_after_sim(event, context):
    if event.returncode != 0:
      raise RuntimeError(
        "Could not confirm Gazebo robot readiness: "
        "Waiting for /joint_states failed."
      )

    return [moveit, frame_translator, dls_teleop]

  wait_for_simulation = RegisterEventHandler(
    OnProcessExit(
      target_action=wait_for_joint_states,
      on_exit=start_teleoperation_after_sim,
    )
  )

  return LaunchDescription([
    gazebo,
    wait_for_simulation,
    wait_for_joint_states,
  ])