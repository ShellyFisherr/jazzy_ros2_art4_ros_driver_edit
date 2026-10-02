# annin_ar4_teleoperator
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

This package provides the basic teleoperation for the Annin Robotics AR4 arm using `Ros 2`, `MoveIt 2`, `libsurvive`, `libsurvive_ros2` and `ros2_control`. It includes a calibration node, along with a DLS based movement command node.

The package is an extension of the [`ar4_ros_driver`](https://github.com/Annin-Robotics/ar4_ros_driver/tree/main) and is intended for experimentation with AR4 teleoperation for both simulation and hardware control.

## Overview
The `annin_ar4_teleoperator` package provides 3-dimensional space tracked teleoperation using the HTCVive 2.0 trackers and lighthouses from Valve. The Annin-Robotics arm in question is the AR4 mk5 arm without an end-effector. It should be compatible with other AR4 models as well, although this remains untested. This package is an extension of the [`ar4_ros_driver`](https://github.com/Annin-Robotics/ar4_ros_driver/tree/main) and is based in `Ros 2`, `MoveIt 2`, `libsurvive`, `libsurvive_ros2` and `ros2_control`.

The main control pipeline is:
```text
libsurvive
    -> libsurvive_ros2
    -> HTCVive tracker
    -> Velocity data
    -> MoveIt robotic joint data
    -> DLS teleoperation controller
    -> ros2 control
    -> AR4 robot / gazebo simulation
```

tf_static = lighthouse position and orientation\
velocity = velocity of tracker WRT the tracker itself
