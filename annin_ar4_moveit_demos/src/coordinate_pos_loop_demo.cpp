/**
 * @file coordinate_pos_loop_demo.cpp
 * @brief A basic ROS2 and MoveIt 2 framework to move the AR4 robotic arm (based on MK.5)
 * 
 * This program provides a framework on how to use ROS2 and MoveIt 2 to connect to the
 * AR4 robotic arm and move it to specified coordinates.
 * It sets up a node, creates a MoveGroupInterface for the arm, plans a trajectory
 * and executes the planned motion
 * 
 * @author Daan Krijnen
 * @date September 07, 2026
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

/**
 * @brief The main function that starts the program.
 * 
 * This sets up the ROS2 environment and prepares it for robot control
 * 
 * @param argc The number of input arguments the program recieves
 * @param argv the list of input arguments the program recieves
 * @return int A number indicating if the program finishes successfully (0) or not
 */

 int main(int argc, char * argv[])
 {
    //Start ROS2
    rclcpp::init(argc, argv);

    //Create the coordinate_pos_loop_demo node
    auto const node = std::make_shared<rclcpp::Node>(
        "coordinate_pos_loop_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    //Create a logger shortcut in the name of the Node to print messages as system runs

    auto const logger = rclcpp::get_logger("coordinate_pos_loop_demo");

    // Create the MoveGroup Interfaces for moveit
    // These interfaces are used to plan and execute movements, set target poses,
    // and perform other motion-related tasks for each respective part of the robot.
    // The use of auto allows the compiler to automatically deduce the type of variable.
    // Source: https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
    
    using moveit::planning_interface::MoveGroupInterface;
    auto arm_group_interface = MoveGroupInterface(node, "ar_manipulator");

    //specify a planning pipeline to be used for further planning
    arm_group_interface.setPlanningPipelineId("ompl");

    //Specify a planner to be used for further planning
    arm_group_interface.setPlannerId("RRTConnectkConfigDefault");

    // Specify the maximum amount of time in seconds to use when planning
    arm_group_interface.setPlanningTime(1.0);

    //Set a scaling factor for optionally reducing maximum velocity. values (0,1]
    arm_group_interface.setMaxVelocityScalingFactor(1.0);

    //Set scaling factory for optionally reducing maximum acceleration. Values (0,1]
    arm_group_interface.setMaxAccelerationScalingFactor(1.0);

    //Display helpful logging messages on terminal
    RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

    //Set a target pose for the end effector in carthesian coordinates
    // rough max reach from center of 0.5m
    auto const arm_target_pose = [&node]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = node->now();
        msg.pose.position.x = 0.2;
        msg.pose.position.y = 0.2;
        msg.pose.position.z = 0.2;
        msg.pose.orientation.x = 1.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 0.0;
        return msg;
    }();
    arm_group_interface.setPoseTarget(arm_target_pose);

    //Create a plan to the target pose
    // This will give 2 thing
    // 1. Planning is succesfull (stored in 'success')
    // 2. The actual motion plan (stored in 'plan')

    auto const [success, plan] = [&arm_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
        return std::make_pair(ok, msg); //so 'ok' is a T/F and 'msg' is the path
    }();

    // Try to execute movement plan if created successfully,
    // else, report an error.
    if (success)
    {
        arm_group_interface.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // shut down ROS 2 cleanly when done
    rclcpp::shutdown();
    return 0;
 }