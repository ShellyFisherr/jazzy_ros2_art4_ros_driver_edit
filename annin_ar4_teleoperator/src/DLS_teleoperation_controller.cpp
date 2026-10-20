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
 *  velocity_subscription_      /velocity_world                         geometry_msgs::msg::TwistStamped    (velocity inputs in world frame)
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <sstream>
#include <iomanip>
#include <array>

//constexpr double CONTROL_DT_ = 1.0 / 60.0; // Timer in seconds . (base = 60 Hz)
//constexpr bool DEBUG_ = true; // enable debug messages.

constexpr double PI = 3.14159265359;

class DlsReducedController : public rclcpp::Node
{
    using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
    using TrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<FollowTrajectory>;
    using SwitchControllers = controller_manager_msgs::srv::SwitchController;
    using JointVector = Eigen::Matrix<double, 6, 1>;

public:
    DlsReducedController(const rclcpp::NodeOptions & node_options)
    : Node("dls_reduced_controller", node_options)
    {

        load_parameters();

        velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/velocity_world", 10, std::bind(&DlsReducedController::velocity_callback, this, std::placeholders::_1));

        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&DlsReducedController::joint_state_callback, this, std::placeholders::_1));

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/position_world", 10, std::bind(&DlsReducedController::pose_callback, this, std::placeholders::_1));

        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_velocity_controller/commands", 10);

        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        controller_switch_client_ = create_client<SwitchControllers>("/controller_manager/switch_controller");

        trajectory_client_ = rclcpp_action::create_client<FollowTrajectory>(this, 
            "/joint_trajectory_controller/follow_joint_trajectory");

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

        robot_model_ = robot_model;

        RCLCPP_INFO(get_logger(), "Robot model retrieved.");

        joint_model_group_ = robot_model->getJointModelGroup("ar_manipulator");
        if (joint_model_group_ == nullptr)
        {
            throw std::runtime_error("Planning group ar_manipulator was not found");
        }

        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model);
        robot_state_->setToDefaultValues();

        //move_group_->startStateMonitor();



        startup_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this, robot_model]()
        {
            bool have_joint_state;
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                have_joint_state = received_joint_state_;
            }

            if (!have_joint_state || !trajectory_client_->action_server_is_ready())
            {
                return;
            }

            startup_timer_->cancel();

            try
            {
                std::cerr << "Launching initial pose movement" << std::endl;
                launch_initial_pose(robot_model);
            }
            catch(const std::exception & ex)
            {
                RCLCPP_ERROR(get_logger(),
                    "Startup pose failed; DLS will remain disabled: %s",
                    ex.what());
            }
            
            
        });
    }


    void stop_motion()
    {
        if (control_timer_)
        {
            control_timer_->cancel();

            publish_zero_velocity();
        }

        if (!controller_switch_client_->service_is_ready())
        {
            RCLCPP_ERROR(get_logger(), "Controller manager is unavailable during stop");
            rclcpp::shutdown();
            return;
        }

        auto request = std::make_shared<SwitchControllers::Request>();
        request->deactivate_controllers = {"joint_velocity_controller"};
        request->strictness = SwitchControllers::Request::BEST_EFFORT;
        request->activate_asap = true;
        request->timeout.sec = 2;
        request->timeout.nanosec = 0;

        controller_switch_client_->async_send_request(request, [this](rclcpp::Client<SwitchControllers>::SharedFuture future)
        {
            const auto response = future.get();
            if (!response->ok)
            {
                RCLCPP_ERROR(get_logger(),
                    "Could not deactivate velocity controller: %s",
                    response->message.c_str());
            }

            rclcpp::shutdown();
        });
    }


private:

    rclcpp::Client<SwitchControllers>:: SharedPtr controller_switch_client_;

    rclcpp_action::Client<FollowTrajectory>::SharedPtr trajectory_client_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;

    rclcpp::TimerBase::SharedPtr startup_timer_;

    moveit::core::RobotModelConstPtr robot_model_;
    std::chrono::steady_clock::time_point joint_state_received_at_;

    sensor_msgs::msg::JointState latest_joint_state_;
    std::mutex joint_state_mutex_;
    bool received_joint_state_ = false;

    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<moveit::core::RobotState> robot_state_;
    const moveit::core::JointModelGroup * joint_model_group_;

    std::vector<double> current_positions;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    rclcpp::Time last_input_time_;
    bool received_input_ = false;

    bool orientation_reference_initialized_ = false;
    Eigen::Matrix3d locked_relative_orientation_;
    Eigen::Matrix3d locked_base_orientation_;

    Eigen::Vector3d robot_near_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d tracker_pose_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d previous_ee_position_;
    std::chrono::steady_clock::time_point previous_ee_time_;
    bool previous_ee_valid_ = false;

    double deadzone_velocity_;
    double deadzone_twist_;

    double CONTROL_DT_;
    bool DEBUG_;

    double translation_scale_;
    double timeout_age_;


    std::mutex velocity_mutex_;

    double vx_ = 0.0;
    double vy_ = 0.0;
    double vz_ = 0.0;
    double wx_ = 0.0;
    double wy_ = 0.0;
    double wz_ = 0.0;

    std::string tracker_frame_;

    std::chrono::steady_clock::time_point last_velocity_time_;
    bool received_velocity_ = false;

    geometry_msgs::msg::PoseStamped latest_tracker_pose_;
    std::mutex tracker_pose_mutex_;
    bool received_tracker_pose_ = false;
    std::chrono::steady_clock::time_point tracker_pose_received_at_;

    bool feedback_reference_set_ = false;
    Eigen::Vector3d tracker_reference_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d robot_reference_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond tracker_reference_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond robot_reference_orientation_ = Eigen::Quaterniond::Identity();

    double position_gain_;
    double orientation_gain_;
    double deadzone_translation_error_;
    double deadzone_orientation_error_;
    double DLS_damping_;

    JointVector previous_commanded_velocity_ = JointVector::Zero();
    std::chrono::steady_clock::time_point previous_command_time_;
    bool velocity_limiter_initialized_ = false;

    std::unordered_map<std::string, double> previous_joint_velocities_;
    std::unordered_map<std::string, double> latest_joint_accelerations_;
    std::chrono::steady_clock::time_point previous_joint_velocity_time_;
    bool have_previous_joint_velocity_sample_ = false;

    bool velocity_limiting_;
    bool acceleration_limiting_;

    std::array<double, 6> max_joint_velocity_;
    std::array<double ,6> max_joint_acceleration_;

    static Eigen::Vector3d vector_from_parameter(
        const std::vector<double> & values,
        const std::string & name)
    {
        if (values.size() != 3)
        {
            throw std::runtime_error(name + " must contain exactly three values.");
        }

        const Eigen::Vector3d result(values[0], values[1], values[2]);

        if (!result.allFinite())
        {
            throw std::runtime_error(name + " values must be finite.");
        }

        return result;
    }

    void load_parameters()
    {
        translation_scale_ = declare_parameter<double>("translation_scale", 1.0);

        if (!std::isfinite(translation_scale_) || translation_scale_ <= 0)
        {
            throw std::runtime_error("translation_scale must be positive and finite.");
        }

        const std::vector<double> robot_near_values =
            declare_parameter<std::vector<double>>("robot_near", std::vector<double>{0.0, 0.0, 0.0});

        if (robot_near_values.size() != 3)
        {
            throw std::runtime_error("robot_near must have exactly 3 values.");
        }

        robot_near_ = vector_from_parameter(robot_near_values, "robot_near");

        deadzone_velocity_ = declare_parameter<double>("deadzone_velocity", 0.002);

        if (!std::isfinite(deadzone_velocity_) || deadzone_velocity_ <= 0.0)
        {
            throw std::runtime_error("deadzone_velocity must be finite and larger or equal to zero");
        }

        deadzone_twist_ = declare_parameter<double>("deadzone_twist", 0.008);

        if (!std::isfinite(deadzone_twist_) || deadzone_twist_ < 0.0)
        {
            throw std::runtime_error("deadzone_twist must be finite and larger or equal to zero");
        }

        CONTROL_DT_ = declare_parameter<double>("CONTROL_DT", 60.0);

        if (!std::isfinite(CONTROL_DT_) || CONTROL_DT_ <= 0.0)
        {
            throw std::runtime_error("controller_frequency must be positive and finite");
        }
        CONTROL_DT_ = 1.0 / CONTROL_DT_;

        DEBUG_ = declare_parameter<bool>("DEBUG", true);

        tracker_frame_ = declare_parameter<std::string>("target_frame", "Unknown");

        if (tracker_frame_.empty())
        {
            throw std::runtime_error("tracker_frame cannot be empty");
        }

        timeout_age_ = declare_parameter<double>("timeout_age", 0.1);

        if (!std::isfinite(timeout_age_) || timeout_age_ <= 0.0)
        {
            throw std::runtime_error("timeout_age must be positive and finite");
        }

        deadzone_translation_error_ = declare_parameter<double>("deadzone_translation_error", 0.0);

        if (!std::isfinite(deadzone_translation_error_) || deadzone_translation_error_ < 0.002)
        {
            throw std::runtime_error("deadzone translation error must be finite and larger or equal to zero");
        }

        deadzone_orientation_error_ = declare_parameter<double>("deadzone_orientation_error", 0.0);

        if (!std::isfinite(deadzone_orientation_error_) || deadzone_orientation_error_ < 0.008)
        {
            throw std::runtime_error("deadzone orientation error must be finite and larger or equal to zero");
        }

        position_gain_ = declare_parameter<double>("position_gain", 1.0);

        if (!std::isfinite(position_gain_) || position_gain_ <= 0.0)
        {
            throw std::runtime_error("position gain must be positive and finite");
        }

        orientation_gain_ = declare_parameter<double>("orientation_gain", 1.0);
        
        if (!std::isfinite(orientation_gain_) || orientation_gain_ <= 0.0)
        {
            throw std::runtime_error("orientation gain must be positive and finite");
        }

        DLS_damping_ = declare_parameter<double>("DLS_damping", 0.0);

        if (!std::isfinite(DLS_damping_) || DLS_damping_ < 0.0)
        {
            throw std::runtime_error("DLS damping must be finite and larger or equal to zero");
        }

        const auto max_velocity = declare_parameter<std::vector<double>>("max_velocity", std::vector<double>(6, 60.0));
        const auto max_acceleration = declare_parameter<std::vector<double>>("max_acceleration", std::vector<double>(6, 30.0));

        if (max_velocity.size() != max_joint_velocity_.size())
        {
            throw std::runtime_error("max_velocity must have six values");
        }

        if (max_acceleration.size() != max_joint_acceleration_.size())
        {
            throw std::runtime_error("max_acceleration must have six values");
        }

        for (std::size_t i = 0; i < max_joint_velocity_.size(); ++i)
        {
            if (!std::isfinite(max_velocity[i]) || max_velocity[i] <= 0.0 ||
                !std::isfinite(max_acceleration[i]) || max_acceleration[i] <= 0.0)
            {
                throw std::runtime_error("max_velocity and max_acceleration must be positive and finite");
            }

            max_joint_velocity_[i] = max_velocity[i] * PI / 180;
            max_joint_acceleration_[i] = max_acceleration[i] * PI / 180;
        }

        velocity_limiting_ = declare_parameter<bool>("velocity_limit", true);
        acceleration_limiting_ = declare_parameter<bool>("acceleration_limit", true);

        std::cerr << "=============================================" << std::endl;
        std::cerr << "              Parameters Loaded" << std::endl;
        std::cerr << "=============================================" << std::endl;
        std::cerr << "   controller timer:           " << CONTROL_DT_ << " [s]" << std::endl;
        std::cerr << "   shows logs:                 " << (DEBUG_ ? "True":"False") << std::endl;
        std::cerr << "   translation scale:          " << translation_scale_ << std::endl;
        std::cerr << "   robot start position:    x: " << robot_near_.x() << " [m]" << std::endl;
        std::cerr << "                            y: " << robot_near_.y() << " [m]" << std::endl;
        std::cerr << "                            z: " << robot_near_.z() << " [m]" << std::endl;
        std::cerr << "   deadzone velocity:          " << deadzone_velocity_ << " [m/s]" << std::endl;
        std::cerr << "   deadzone twist:             " << deadzone_twist_ << " [rad/s]" << std::endl;
        std::cerr << "   tracker frame:              " << tracker_frame_ << std::endl;
        std::cerr << "   timeout age:                " << timeout_age_ << " [m/s]" << std::endl;
        std::cerr << "   deadzone translation error: " << deadzone_translation_error_ << " [m]" << std::endl;
        std::cerr << "   deadzone orientation error: " << deadzone_orientation_error_ << " [rad]" << std::endl;
        std::cerr << "   position gain:              " << position_gain_ << std::endl;
        std::cerr << "   orientation gain:           " << orientation_gain_ << std::endl;
        std::cerr << "   DLS damping coefficient:    " << DLS_damping_ << std::endl;
        std::cerr << "   Limit joint velocities:     " << (velocity_limiting_ ? "True":"False") << std::endl;
        std::cerr << "   Max joint velocities:   J1: " << max_joint_velocity_[0] << " [rad/s]" << std::endl;
        std::cerr << "                           J2: " << max_joint_velocity_[1] << " [rad/s]" << std::endl;
        std::cerr << "                           J3: " << max_joint_velocity_[2] << " [rad/s]" << std::endl;
        std::cerr << "                           J4: " << max_joint_velocity_[3] << " [rad/s]" << std::endl;
        std::cerr << "                           J5: " << max_joint_velocity_[4] << " [rad/s]" << std::endl;
        std::cerr << "                           J6: " << max_joint_velocity_[5] << " [rad/s]" << std::endl;
        std::cerr << "   Limit joint accelerations:  " << (acceleration_limiting_ ? "True":"False") << std::endl;
        std::cerr << "   Max joint acceleration: J1: " << max_joint_acceleration_[0] << " [rad/s^2]" << std::endl;
        std::cerr << "                           J2: " << max_joint_acceleration_[1] << " [rad/s^2]" << std::endl;
        std::cerr << "                           J3: " << max_joint_acceleration_[2] << " [rad/s^2]" << std::endl;
        std::cerr << "                           J4: " << max_joint_acceleration_[3] << " [rad/s^2]" << std::endl;
        std::cerr << "                           J5: " << max_joint_acceleration_[4] << " [rad/s^2]" << std::endl;
        std::cerr << "                           J6: " << max_joint_acceleration_[5] << " [rad/s^2]" << std::endl;
        std::cerr << "=============================================" << std::endl;
    }

    void launch_initial_pose(const moveit::core::RobotModelConstPtr & robot_model)
    {
        sensor_msgs::msg::JointState joint_state;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            if (!received_joint_state_)
            {
                return;
            }
            joint_state = latest_joint_state_;
        }

        std::unordered_map<std::string, double> position_by_name;
        for (std::size_t i = 0; i < joint_state.name.size(); ++i)
        {
            if (i < joint_state.position.size())
            {
                position_by_name[joint_state.name[i]] = joint_state.position[i];
            }
        }

        std::vector<double> measured_positions;
        for (const auto & joint_name : joint_model_group_->getActiveJointModelNames())
        {
            const auto it = position_by_name.find(joint_name);
            if (it == position_by_name.end())
            {
                throw std::runtime_error("Joint state missing joint: " + joint_name);
            }
            measured_positions.push_back(it->second);
        }


        moveit::core::RobotState goal_state(robot_model);
        goal_state.setJointGroupPositions(joint_model_group_, measured_positions);
        goal_state.update();

        const auto * end_effector = robot_model->getLinkModel("ee_link");

        if (end_effector == nullptr)
        {
            throw std::runtime_error("ee_link not found");
        }

        Eigen::Isometry3d target_pose = goal_state.getGlobalLinkTransform(end_effector);

        target_pose.translation() = robot_near_;

        if (!goal_state.setFromIK(
            joint_model_group_,
            target_pose,
            "ee_link",
            1.0))
        {
            throw std::runtime_error("No IK solution found for robot_near: [" +
            std::to_string(robot_near_.x()) + ", " +
            std::to_string(robot_near_.y()) + ", " +
            std::to_string(robot_near_.z()) + "]");
        }

        std::vector<double> goal_positions;
        goal_state.copyJointGroupPositions(joint_model_group_, goal_positions);

        FollowTrajectory::Goal goal;
        goal.trajectory.joint_names = joint_model_group_->getActiveJointModelNames();

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = goal_positions;
        point.velocities.assign(goal_positions.size(), 0.0);
        point.time_from_start.sec = 4;
        point.time_from_start.nanosec = 0;

        goal.trajectory.points.push_back(point);

        rclcpp_action::Client<FollowTrajectory>::SendGoalOptions options;

        options.goal_response_callback = [this](const TrajectoryGoalHandle::SharedPtr & handle)
        {
            if (!handle)
            {
                RCLCPP_ERROR(get_logger(), "Trajectory goal was rejected");
                return;
            }

            RCLCPP_INFO(get_logger(), "trajectory goal accepted");
        };

        options.result_callback = [this](const TrajectoryGoalHandle::WrappedResult & result)
        {
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || result.result->error_code != FollowTrajectory::Result::SUCCESSFUL)
            {
                RCLCPP_ERROR(get_logger(), "Startup trajectory did not complete successfully");
                return;
            }

            RCLCPP_INFO(get_logger(), "Startup trajectory completed");
            switch_to_velocity_controller();
        };

        trajectory_client_->async_send_goal(goal, options);
    }

    void switch_to_velocity_controller()
    {
        auto request = std::make_shared<SwitchControllers::Request>();
        request->activate_controllers = {"joint_velocity_controller"};
        request->deactivate_controllers = {"joint_trajectory_controller"};
        request->strictness = SwitchControllers::Request::STRICT;
        request->activate_asap = true;
        request->timeout.sec = 2;
        request->timeout.nanosec = 0;

        controller_switch_client_->async_send_request(request, [this](rclcpp::Client<SwitchControllers>::SharedFuture future)
        {
            const auto response = future.get();
            if (!response->ok)
            {
                RCLCPP_ERROR(get_logger(),
                    "Controller switch failed: %s",
                    response->message.c_str());
                return;
            }

            control_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(CONTROL_DT_),
                std::bind(&DlsReducedController::control_loop, this), timer_callback_group_);

            std::cerr << "====================================================" << std::endl;
            std::cerr << "        Controller Activated. Awaiting input" << std::endl;
            std::cerr << "====================================================" << std::endl;

        });
    }


    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (msg->header.frame_id != tracker_frame_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Ignoring tracker pose in frame '%s'; expected '%s'",
                msg->header.frame_id.c_str(),
                tracker_frame_.c_str());

            return;
        }

        {
            std::lock_guard<std::mutex> lock(tracker_pose_mutex_);
            latest_tracker_pose_ = *msg;
            tracker_pose_received_at_ = std::chrono::steady_clock::now();
            received_tracker_pose_ = true;
        }
    }


    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const auto now = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        latest_joint_state_ = *msg;
        received_joint_state_ = true;
        joint_state_received_at_ = now;

        if (DEBUG_)
        {   
            latest_joint_accelerations_.clear();

            bool valid_sample = msg->velocity.size() == msg->name.size();

            for (const double velocity : msg->velocity)
            {
                valid_sample = valid_sample && std::isfinite(velocity);
            }

            if (!valid_sample)
            {
                previous_joint_velocities_.clear();
                have_previous_joint_velocity_sample_ = false;
                return;
            }

            if (have_previous_joint_velocity_sample_)
            {
                const double dt = std::chrono::duration<double>(now - previous_joint_velocity_time_).count();

                for (std::size_t i = 0; i < msg->name.size(); ++i)
                {
                    const auto previous = previous_joint_velocities_.find(msg->name[i]);

                    if (previous != previous_joint_velocities_.end())
                    {
                        latest_joint_accelerations_[msg->name[i]] = (msg->velocity[i] - previous->second) / dt;
                    }
                }
            }

            previous_joint_velocities_.clear();
            for (std::size_t i = 0; i < msg->name.size(); ++i)
            {
                previous_joint_velocities_[msg->name[i]] = msg->velocity[i];
            }

            previous_joint_velocity_time_ = now;
            have_previous_joint_velocity_sample_ = true;
        }
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

        last_velocity_time_ = std::chrono::steady_clock::now();
        received_velocity_ = true;

        if (DEBUG_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Linear: vx=%.6f vy=%.6f vz=%.6f | Angular wx=%.6f wy=%.6f wz=%.6f",
                vx_, vy_, vz_, wx_, wy_, wz_);
        }
    }


    bool get_tracker_pose(geometry_msgs::msg::PoseStamped & pose)
    {
        std::lock_guard<std::mutex> lock(tracker_pose_mutex_);

        const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - tracker_pose_received_at_).count();

        if (!received_tracker_pose_ || age > timeout_age_)
        {
            return false;
        }

        pose = latest_tracker_pose_;
        return true;
    }


    bool get_robot_pose(
        geometry_msgs::msg::PoseStamped & pose,
        sensor_msgs::msg::JointState & joint_state,
    std::vector<double> & positions)
    {
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);

            const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - joint_state_received_at_).count();

            if (!received_joint_state_ || age > timeout_age_)
            {
                return false;
            }
            joint_state = latest_joint_state_;
        }        

        std::unordered_map<std::string, double> by_name;
        for (std::size_t i = 0; i < joint_state.name.size(); ++i)
        {
            if (i < joint_state.position.size())
            {
                by_name[joint_state.name[i]] = joint_state.position[i];
            }
        }

        positions.clear();
        for (const auto & name : joint_model_group_->getActiveJointModelNames())
        {
            const auto it = by_name.find(name);
            if (it == by_name.end())
            {
                return false;
            }
            positions.push_back(it->second);
        }

        moveit::core::RobotState state(robot_model_);
        state.setJointGroupPositions(joint_model_group_, positions);
        state.update();

        const auto * ee = robot_model_->getLinkModel("ee_link");
        if (!ee)
        {
            return false;
        }

        const Eigen::Isometry3d transform = state.getGlobalLinkTransform(ee);
        const Eigen::Quaterniond orientation(transform.rotation());

        pose.header.stamp = joint_state.header.stamp;
        pose.header.frame_id = robot_model_->getModelFrame();
        pose.pose.position.x = transform.translation().x();
        pose.pose.position.y = transform.translation().y();
        pose.pose.position.z = transform.translation().z();
        pose.pose.orientation.x = orientation.x();
        pose.pose.orientation.y = orientation.y();
        pose.pose.orientation.z = orientation.z();
        pose.pose.orientation.w = orientation.w();
        
        return true;
    }

    JointVector limit_joint_velocity(const JointVector & requested)
    {
        const auto now = std::chrono::steady_clock::now();

        double dt = 0.0;
        if (velocity_limiter_initialized_)
        {
            dt = std::chrono::duration<double>(now - previous_command_time_).count();
        }

        previous_command_time_ = now;

        if (!velocity_limiter_initialized_)
        {
            velocity_limiter_initialized_ = true;
            previous_commanded_velocity_.setZero();

            if (acceleration_limiting_)
            {
                return previous_commanded_velocity_;
            }
        }

        JointVector target = requested;

        if (velocity_limiting_)
        {
            double velocity_scale = 1.0;

            for (Eigen::Index i = 0; i < 6; ++i)
            {
                const double requested_speed = std::abs(requested[i]);
                if (requested_speed > max_joint_velocity_[i])
                {
                    velocity_scale = std::min(velocity_scale, max_joint_velocity_[i] / requested_speed);
                }
            }

            target *= velocity_scale;
        }

        JointVector limited = target;

        if (acceleration_limiting_)
        {
            if (!std::isfinite(dt) || dt <= 0.0)
            {
                return previous_commanded_velocity_;
            }

            dt = std::min(dt, CONTROL_DT_);

            const JointVector change = target - previous_commanded_velocity_;
            double acceleration_scale = 1.0;

            for (Eigen::Index i = 0; i < 6; ++i)
            {
                const double requested_change = std::abs(change[i]);
                if (requested_change > 0.0)
                {
                    acceleration_scale = std::min(acceleration_scale, max_joint_acceleration_[i] * dt / requested_change);
                }
            }

            acceleration_scale = std::clamp(acceleration_scale, 0.0, 1.0);
            limited = previous_commanded_velocity_ + acceleration_scale * change;
        }

        previous_commanded_velocity_ = limited;
        return limited;
    }

    void control_loop()
    {
        geometry_msgs::msg::PoseStamped tracker_pose;
        geometry_msgs::msg::PoseStamped robot_pose;
        sensor_msgs::msg::JointState joint_state;
        std::vector<double> current_positions;

        if (!get_tracker_pose(tracker_pose) || !get_robot_pose(robot_pose, joint_state, current_positions))
        {
            publish_zero_velocity();
            return;
        }

        const Eigen::Vector3d tracker_position(
            tracker_pose.pose.position.x,
            tracker_pose.pose.position.y,
            tracker_pose.pose.position.z);

        const Eigen::Vector3d robot_position(
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
            robot_pose.pose.position.z);

        Eigen::Quaterniond tracker_orientation(
            tracker_pose.pose.orientation.w,
            tracker_pose.pose.orientation.x,
            tracker_pose.pose.orientation.y,
            tracker_pose.pose.orientation.z);

        Eigen::Quaterniond robot_orientation(
            robot_pose.pose.orientation.w,
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z);

        if (!tracker_position.allFinite() || !robot_position.allFinite() || 
            !tracker_orientation.coeffs().allFinite() || !robot_orientation.coeffs().allFinite())
        {
            publish_zero_velocity();
            return;
        }

        tracker_orientation.normalize();
        robot_orientation.normalize();

        if (!feedback_reference_set_)
        {
            tracker_reference_position_ = tracker_position;
            robot_reference_position_ = robot_position;
            tracker_reference_orientation_ = tracker_orientation;
            robot_reference_orientation_ = robot_orientation;
            feedback_reference_set_ = true;
            publish_zero_velocity();
            return;
        }


        const Eigen::Vector3d tracker_delta = tracker_position - tracker_reference_position_;

        const Eigen::Vector3d target_position = robot_reference_position_ + translation_scale_ * tracker_delta;

        const Eigen::Vector3d position_error = target_position - robot_position;

        Eigen::Vector3d error_outside = Eigen::Vector3d::Zero();
        const double error_magnitude = position_error.norm();

        if (error_magnitude > deadzone_translation_error_)
        {
            error_outside = position_error * ((error_magnitude - deadzone_translation_error_) / error_magnitude);


        }

        

        const Eigen::Vector3d correction_velocity = position_gain_ * error_outside;

        Eigen::Quaterniond tracker_orientation_delta = tracker_orientation * tracker_reference_orientation_.conjugate();
        tracker_orientation_delta.normalize();

        Eigen::Quaterniond target_orientation = tracker_orientation_delta * robot_reference_orientation_;
        target_orientation.normalize();

        Eigen::Quaterniond orientation_error = target_orientation * robot_orientation.conjugate();
        orientation_error.normalize();

        if (orientation_error.w() < 0.0)
        {
            orientation_error.coeffs() *= -1.0;
        }

        const double sin_half_angle = orientation_error.vec().norm();
        Eigen::Vector3d angular_error = Eigen::Vector3d::Zero();

        if (sin_half_angle > 0.0)
        {
            const double angle = 2.0 * std::atan2(sin_half_angle, orientation_error.w());

            if (angle > deadzone_orientation_error_)
            {
                angular_error = ((angle - deadzone_orientation_error_) / sin_half_angle) * orientation_error.vec();
            }
        }

        const Eigen::Vector3d angular_correction = orientation_gain_ * angular_error;

        Eigen::Vector3d tracker_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d tracker_angular_velocity = Eigen::Vector3d::Zero();

        {
            std::lock_guard<std::mutex> lock(velocity_mutex_);

            const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_velocity_time_).count();
            
            if (received_velocity_ && age <= timeout_age_)
            {
                tracker_velocity = Eigen::Vector3d(vx_, vy_, vz_);
                tracker_angular_velocity = Eigen::Vector3d(wx_, wy_, wz_);

                for (Eigen::Index j = 0; j < 3; ++j)
                {
                    if (std::abs(tracker_velocity[j]) < deadzone_velocity_)
                    {
                        tracker_velocity[j] = 0.0;
                    }

                    if (std::abs(tracker_angular_velocity[j]) < deadzone_twist_)
                    {
                        tracker_angular_velocity[j] = 0.0;
                    }
                }
            }
        }

        const Eigen::Vector3d feedforward_velocity = translation_scale_ * tracker_velocity;

        const Eigen::Vector3d final_linear_velocity = feedforward_velocity + correction_velocity;

        const Eigen::Vector3d final_angular_velocity = tracker_angular_velocity + angular_correction;

        JointVector desired_twist;
        desired_twist << 
            final_linear_velocity.x(),
            final_linear_velocity.y(),
            final_linear_velocity.z(),
            final_angular_velocity.x(),
            final_angular_velocity.y(),
            final_angular_velocity.z();

        robot_state_->setJointGroupPositions(joint_model_group_, current_positions);
        robot_state_->update();

        const auto * end_effector = robot_model_->getLinkModel("ee_link");
        if (end_effector == nullptr)
        {
            publish_zero_velocity();
            return;
        }

        Eigen::MatrixXd jacobian;
        robot_state_->getJacobian(
            joint_model_group_,
            end_effector,
            Eigen::Vector3d::Zero(),
            jacobian);

        if (jacobian.rows() != 6 || jacobian.cols() != 6)
        {
            publish_zero_velocity();
            return;
        }

        const Eigen::Matrix<double, 6, 6> regularized =
            jacobian * jacobian.transpose() + DLS_damping_ * DLS_damping_ * Eigen::Matrix<double, 6, 6>::Identity();

        const JointVector joint_velocity = jacobian.transpose() * regularized.ldlt().solve(desired_twist);

        if (!joint_velocity.allFinite())
        {
            publish_zero_velocity();
            return;
        }

        const JointVector limited_joint_velocity = limit_joint_velocity(joint_velocity);

        

        if (DEBUG_)
        {
            const auto & joint_names = joint_model_group_->getActiveJointModelNames();

            std::unordered_map<std::string, double> measured_velocity_by_name;
            for (std::size_t i = 0; i < joint_state.name.size() && i < joint_state.velocity.size(); ++i)
            {
                measured_velocity_by_name[joint_state.name[i]] = joint_state.velocity[i];
            }

            std::vector<double> actual_joint_velocities;
            actual_joint_velocities.reserve(joint_names.size());

            bool have_all_actual_velocities = true;
            for (const auto & joint_name : joint_names)
            {
                const auto it = measured_velocity_by_name.find(joint_name);
                if (it == measured_velocity_by_name.end())
                {
                    have_all_actual_velocities = false;
                    break;
                }

                actual_joint_velocities.push_back(it->second);
            }

            std::unordered_map<std::string, double> actual_accelerations;
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                actual_accelerations = latest_joint_accelerations_;
            }

            RCLCPP_INFO(get_logger(),
            "Tracker p[m]=[%.3f %.3f %.3f] q[xyzw]=[%.3f %.3f %.3f %.3f] | "
            "Robot p[m]=[%.3f %.3f %.3f] q[xyzw]=[%.3f %.3f %.3f %.3f] | "
            "Error p[m]=[%.3f %.3f %.3f] r[rad]=[%.3f %.3f %.3f] | "
            "FF v[m/s]=[%.3f %.3f %.3f] w[rad/s]=[%.3f %.3f %.3f] | "
            "Correction v[m/s]=[%.3f %.3f %.3f] w[rad/s]=[%.3f %.3f %.3f] | "
            "Final v[m/s]=[%.3f %.3f %.3f] w[rad/s]=[%.3f %.3f %.3f]",
            tracker_position.x(), tracker_position.y(), tracker_position.z(),
            tracker_orientation.x(), tracker_orientation.y(), tracker_orientation.z(), tracker_orientation.w(),
            robot_position.x(), robot_position.y(), robot_position.z(),
            robot_orientation.x(), robot_orientation.y(), robot_orientation.z(), robot_orientation.w(),
            position_error.x(), position_error.y(), position_error.z(),
            angular_error.x(), angular_error.y(), angular_error.z(),
            feedforward_velocity.x(), feedforward_velocity.y(), feedforward_velocity.z(),
            tracker_angular_velocity.x(), tracker_angular_velocity.y(), tracker_angular_velocity.z(),
            correction_velocity.x(), correction_velocity.y(), correction_velocity.z(),
            angular_correction.x(), angular_correction.y(), angular_correction.z(),
            final_linear_velocity.x(), final_linear_velocity.y(), final_linear_velocity.z(),
            final_angular_velocity.x(), final_angular_velocity.y(), final_angular_velocity.z());

            if (have_all_actual_velocities && static_cast<std::size_t>(joint_velocity.size()) == joint_names.size())
            {
                std::ostringstream velocity_log;
                velocity_log << std::fixed << std::setprecision(3);
                for (std::size_t i = 0; i < joint_names.size(); ++i)
                {
                    if (i > 0)
                    {
                        velocity_log << " | ";
                    }

                    velocity_log << joint_names[i]
                                 << " requested=" << joint_velocity[i]
                                 << " actual=" << actual_joint_velocities[i];

                    const auto acceleration = actual_accelerations.find(joint_names[i]);

                    if (acceleration != actual_accelerations.end())
                    {
                        velocity_log << " actual_eccel=" << acceleration->second;
                    } 
                    else
                    {
                        velocity_log << " actual_eccel=n/a";
                    }
                }

                const std::string velocity_log_text = velocity_log.str();

                RCLCPP_INFO(get_logger(),
                    "Joint velocities [rad/s]: %s",
                    velocity_log_text.c_str());
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Measured joint velocities are missing from /joint_states");
            }
            

        }

       

        std_msgs::msg::Float64MultiArray command;
        command.data.resize(joint_velocity.size());

        for (Eigen::Index i=0; i < limited_joint_velocity.size(); ++i)
            {
                command.data[static_cast<std::size_t>(i)] = limited_joint_velocity[i];
            }
        
        velocity_publisher_->publish(command);
    }

    void publish_zero_velocity()
    {

        previous_commanded_velocity_.setZero();
        previous_command_time_ = std::chrono::steady_clock::now();
        velocity_limiter_initialized_ = true;

        std_msgs::msg::Float64MultiArray command;
        command.data.assign(6, 0.0);
        velocity_publisher_->publish(command);

    }

};

int main(int argc, char * argv[])
{
    std::cerr << "========== DLS EXECUTABLE STARTED ==========" << std::endl;

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    //node_options.automatically_declare_parameters_from_overrides(true);
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

    node->stop_motion();

    rclcpp::shutdown();
    return 0;
}