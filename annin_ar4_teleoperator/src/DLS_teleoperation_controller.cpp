/**
 * @file DLS_teleoperation_controller.cpp
 * @brief A more advanced ROS2 and MoveIt 2 framework to move the AR4 robotic arm (based on MK.5)
 * 
 * This program provides a framework on how to use ROS2 and MoveIt 2 to connect to the
 * AR4 robotic arm and move it with continuous movement. It will use DLS and null-space 
 * controll to try and deal with singularities.
 * It sets up a node, and uses twist to directly move the robot, bypassing planning
 * 
 * -------------------------
 * Publishing Topics
 *  velocity_publisher_         /joint_velocity_controller/commands     std_msgs::msg::Float64MultiArray    (sends velocty commands for the joints)
 * 
 * -------------------------
 * Subscribed Topics
 *  
 *  joint_state_subscriber_     /joint_states                           sensor_msgs::msg::JointState        (takes the joint positions of the AR4 bot)
 * 
 * @author Daan Krijnen
 * @date September 10, 2026
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <vector>
#include <thread>
#include <Eigen/Dense>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <unordered_map>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

constexpr double CONTROL_DT = 1.0 / 60.0; // Timer in seconds . (base = 60 Hz)
constexpr bool DEBUG = false; // enable debug messages.

class DlsReducedController : public rclcpp::Node
{
public:

    DlsReducedController(const rclcpp::NodeOptions & node_options)
    : Node("dls_reduced_controller", node_options)
    {
        velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/velocity_world", 10, std::bind(&DlsReducedController::velocity_callback, this, std::placeholders::_1));

        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&DlsReducedController::joint_state_callback, this, std::placeholders::_1));

        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_velocity_controller/commands", 10);

        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    
    void initialize_system()
    {
        RCLCPP_INFO(get_logger(), "Loading robot model...");

        robot_model_loader::RobotModelLoader robot_model_loader(
            shared_from_this(), "robot_description");
        const auto robot_model = robot_model_loader.getModel();

        if (!robot_model)
        {
            throw std::runtime_error("Moveit robot model is unavailable");
        }

        RCLCPP_INFO(get_logger(), "Robot model retrieved.");

        joint_model_group_ = robot_model->getJointModelGroup("ar_manipulator");
        if (joint_model_group_ == nullptr)
        {
            throw std::runtime_error("Planning group ar_manipulator was not found");
        }

        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model);
        robot_state_->setToDefaultValues();

        //move_group_->startStateMonitor();

        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(CONTROL_DT),
            std::bind(&DlsReducedController::control_loop, this), timer_callback_group_);

        RCLCPP_INFO(get_logger(), "DLS control timer created");
    }

private:
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;

    sensor_msgs::msg::JointState latest_joint_state_;
    std::mutex joint_state_mutex_;
    bool received_joint_state_ = false;

    std::string pressed_key_;
    std::mutex input_mutex_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<moveit::core::RobotState> robot_state_;
    const moveit::core::JointModelGroup * joint_model_group_;

    std::vector<double> current_positions;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    rclcpp::Time last_input_time_;
    bool received_input_ = false;

    bool orientation_reference_initialized_ = false;
    Eigen::Matrix3d locked_relative_orientation_;


    Eigen::Vector3d previous_ee_position_;
    rclcpp::Time previous_ee_time_;
    bool previous_ee_valid_ = false;

    double orientation_gain_ = 1.0;  
    double max_pitch_correction_vel_ = 0.4;
    double deadzone_velocity_ = 0.001;      // m/s
    double deadzone_twist_ = 0.02;          // rad/s

    std::mutex velocity_mutex_;

    double vx_ = 0.0;
    double vy_ = 0.0;
    double vz_ = 0.0;
    double wx_ = 0.0;
    double wy_ = 0.0;
    double wz_ = 0.0;

    rclcpp::Time last_velocity_time_;
    bool received_velocity_ = false;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        latest_joint_state_ = *msg;
        received_joint_state_ = true;
    }

    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);

        vx_ = msg->twist.linear.x;
        vy_ = msg->twist.linear.y;
        vz_ = msg->twist.linear.z;

        wx_ = msg->twist.angular.x;
        wy_ = msg->twist.angular.y;
        wz_ = msg->twist.angular.z;

        last_velocity_time_ = this->now();
        received_velocity_ = true;

        if (DEBUG)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Linear: vx=%.6f vy=%.6f vz=%.6f | Angular wx=%.6f wy=%.6f wz=%.6f",
                vx_, vy_, vz_, wx_, wy_, wz_);
        }
    }

    void control_loop()
    {
        RCLCPP_INFO_ONCE(get_logger(), "DLS control loop entered");


        double velocity_x = 0.0;
        double velocity_y = 0.0;
        double velocity_z = 0.0;
        double omega_x = 0.0;
        double omega_y = 0.0;
        double omega_z = 0.0;

        {
            std::lock_guard<std::mutex> lock(velocity_mutex_);

            if (!received_velocity_)
            {
                return;
            }

            const double age = (this->now() - last_velocity_time_).seconds();

            if (age > 0.1)
            {
                velocity_x = 0.0;
                velocity_y = 0.0;
                velocity_z = 0.0;
                omega_x = 0.0;
                omega_y = 0.0;
                omega_z = 0.0;
            }
            else
            {
                velocity_x = (std::abs(vx_) > deadzone_velocity_) ? -vx_ : 0.0;

                velocity_y = (std::abs(vy_) > deadzone_velocity_) ? -vy_ : 0.0;

                velocity_z = (std::abs(vz_) > deadzone_velocity_) ? vz_ : 0.0;

                //omega_x = (std::abs(wx_) > deadzone_twist_) ? wx_ : 0.0;

                //omega_y = (std::abs(wy_) > deadzone_twist_) ? wy_ : 0.0;

                //omega_z = (std::abs(wz_) > deadzone_twist_) ? wz_ : 0.0;
            }
        }
        

        std::string pressed_key;
        rclcpp::Time last_input_time;

        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            pressed_key = pressed_key_;
        }
     
        sensor_msgs::msg::JointState joint_state_copy;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);

            if (!received_joint_state_)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Waiting for joint states");

                    return;
            }
            
            joint_state_copy = latest_joint_state_;
        }

        std::unordered_map<std::string, double> position_by_name;

        for (std::size_t index = 0; index < joint_state_copy.name.size(); ++index)
        {
            if (index < joint_state_copy.position.size())
            {
                position_by_name[joint_state_copy.name[index]] = 
                    joint_state_copy.position[index];
            }
        }

        current_positions.clear();

        for (const std::string & joint_name : joint_model_group_->getActiveJointModelNames())
        {
            auto position = position_by_name.find(joint_name);

            if (position == position_by_name.end())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Joint state missing joint: %s",
                    joint_name.c_str());

                return;
            }

            current_positions.push_back(position->second);
        }

        robot_state_->setJointGroupPositions(joint_model_group_, current_positions);
        robot_state_->update();

        const auto * end_effector = robot_state_->getLinkModel("ee_link");
        const auto * forearm_link = robot_state_->getLinkModel("link_3");
        

        if (end_effector == nullptr)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "End-effector link ee_link was not found");

            return;
        }

        if (forearm_link == nullptr)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Forearm link link_3 was not found");

            return;
        }

        Eigen::Isometry3d ee_transform = robot_state_->getGlobalLinkTransform(end_effector);

        Eigen::Isometry3d forearm_transform = robot_state_->getGlobalLinkTransform(forearm_link);

        Eigen::Matrix3d current_relative_orientation = forearm_transform.rotation().transpose() * ee_transform.rotation();

        if (!orientation_reference_initialized_)
        {
            locked_relative_orientation_ = forearm_transform.rotation().transpose() * ee_transform.rotation();

            orientation_reference_initialized_ = true;
        }

        Eigen::Vector3d ee_position = ee_transform.translation(); 

        if (previous_ee_valid_)
        {
            double dt = (this->now() - previous_ee_time_).seconds();

            if (dt > 1e-6)
            {
                Eigen::Vector3d actual_velocity = (ee_position - previous_ee_position_) / dt;
            
                if (DEBUG)
                {
                    RCLCPP_INFO_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        1000,
                        "Actual EE velocity: x= %.5f y=%.5f z=%.5f",
                        actual_velocity.x(), actual_velocity.y(), actual_velocity.z());
                }
            }
        }

        previous_ee_position_ = ee_position;
        previous_ee_time_ = this->now();
        previous_ee_valid_ = true;

        //get jacobians

        Eigen::MatrixXd full_jacobian;
        robot_state_->getJacobian(
            joint_model_group_,
            end_effector,
            Eigen::Vector3d::Zero(),
            full_jacobian);

        if (full_jacobian.rows() !=6 || full_jacobian.cols() != 6)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Expected a 6x6 Jacobian got %ldx%ld",
                full_jacobian.rows(),
                full_jacobian.cols());

            return;
        }

        Eigen::Matrix<double, 3, 6> linear_jacobian = full_jacobian.topRows<3>();
        Eigen::Matrix<double, 3, 6> angular_jacobian = full_jacobian.bottomRows<3>();

        Eigen::Matrix3d relative_error = locked_relative_orientation_ * current_relative_orientation.transpose();

        Eigen::AngleAxisd error_angle_axis(relative_error);
        Eigen::Vector3d rotation_error = error_angle_axis.angle() * error_angle_axis.axis();

        Eigen::Vector3d pitch_axis = Eigen::Vector3d::UnitY();

        double pitch_error = pitch_axis.dot(rotation_error);

        double pitch_correction = orientation_gain_ * pitch_error;
        pitch_correction = std::clamp(pitch_correction, -max_pitch_correction_vel_, max_pitch_correction_vel_);

        Eigen::MatrixXd forearm_jacobian;
        robot_state_->getJacobian(
            joint_model_group_,
            forearm_link,
            Eigen::Vector3d::Zero(),
            forearm_jacobian);

        if (forearm_jacobian.rows() != 6 || forearm_jacobian.cols() != 6)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Expected a 6x6 forearm Jacobian");

            return;
        }
        
        Eigen::Matrix<double, 3, 6> forearm_angular_jacobian = forearm_jacobian.bottomRows<3>();

        Eigen::Matrix<double, 3, 6> relative_angular_jacobian = forearm_transform.rotation().transpose() * (angular_jacobian - forearm_angular_jacobian);

        if (DEBUG)
        {
            RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(), 1000,
                "Desired velocity: x=%.3f y=%.3f z=%.3f OmegaX=%.3f OmegaY=%.3f OmegaZ=%.3f",
                velocity_x, velocity_y, velocity_z, omega_x, omega_y, omega_z);
        }

        //create desired DLS joint velocity
        Eigen::Matrix<double, 6, 1> desired_twist(
            velocity_x,
            velocity_y,
            velocity_z,
            omega_x,
            omega_y,
            omega_z);

        Eigen::Matrix<double, 6, 6> task_jacobian;
        task_jacobian.topRows<3>() = linear_jacobian;
        task_jacobian.bottomRows<3>() = relative_angular_jacobian;




        double damping_ = 0.005;
 
        Eigen::Matrix<double, 6, 6> damping_matrix = damping_ * damping_ * Eigen::Matrix<double, 6, 6>::Identity();

        Eigen::Matrix<double, 6, 6> dls_inverse = task_jacobian.transpose() * (task_jacobian * task_jacobian.transpose() + damping_matrix).inverse();

        Eigen::Matrix<double, 6, 1> joint_velocity = dls_inverse * desired_twist;

        Eigen::Matrix<double, 6, 1> achieved_twist = task_jacobian * joint_velocity;

        if (DEBUG)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Desired linear: %.4f %.4f %.4f | Achieved linear: %.4f %.4f %.4f",
                desired_twist(0),
                desired_twist(1),
                desired_twist(2),
                achieved_twist(0),
                achieved_twist(1),
                achieved_twist(2));

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Desired angular: %.4f %.4f %.4f | Achieved angular: %.4f %.4f %.4f",
                desired_twist(3),
                desired_twist(4),
                desired_twist(5),
                achieved_twist(3),
                achieved_twist(4),
                achieved_twist(5));




            RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Joint velocities: %.3f %.3f %.3f %.3f %.3f %.3f",
            joint_velocity(0),
            joint_velocity(1),
            joint_velocity(2),
            joint_velocity(3),
            joint_velocity(4),
            joint_velocity(5));
        }
        
        //publish
        std_msgs::msg::Float64MultiArray command;
        command.data.resize(joint_velocity.size());

        for (Eigen::Index idx = 0; idx < joint_velocity.size(); ++idx)
        {

            command.data[idx] = joint_velocity(idx);
        }

        velocity_publisher_->publish(command);
    }
};

int main(int argc, char * argv[])
{
    std::cerr << "========== DLS EXECUTABLE STARTED ==========" << std::endl;

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    node_options.automatically_declare_parameters_from_overrides(true);
    //node_options.append_parameter_override("use_sim_time", true);

    auto node = std::make_shared<DlsReducedController>(node_options);

    

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node);
    
    std::cerr << "========== CALLING INITIALIZE_SYSTEM ==========" << std::endl;

    node->initialize_system();

    std::cerr << "========== INITIALIZE_SYSTEM FINISHED ==========" << std::endl;

    executor.spin();

    // executor.cancel();
    // executor_thread.join();

    rclcpp::shutdown();
    return 0;
}