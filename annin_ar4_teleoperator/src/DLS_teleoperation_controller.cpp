/**
 * @file DLS_teleoperation_controller.cpp
 * @brief A more advanced ROS2 and MoveIt 2 framework to move the AR4 robotic arm (based on MK.5)
 * 
 * This program provides a framework on how to use ROS2 and MoveIt 2 to connect to the
 * AR4 robotic arm and move it with continuous movement.
 * The controller maps tracker motion and pose error to joint velocity commands
 * using a weighted damped least-squares QP with joint velocity and acceleration
 * constraints.
 * It sets up a node, and uses twist to directly move the robot, bypassing planning
 * 
 * -------------------------
 * Publishing Topics
 *  velocity_publisher_         /joint_velocity_controller/commands     std_msgs::msg::Float64MultiArray    (sends velocity commands for the joints)
 * 
 * -------------------------
 * Subscribed Topics
 *  
 *  joint_state_subscriber_     /joint_states                           sensor_msgs::msg::JointState        (takes the joint positions of the AR4 bot)
 *  velocity_subscription_      /velocity_world                         geometry_msgs::msg::TwistStamped    (velocity inputs in world frame)
 *  pose_subscriber_            /pose_world                             geometry_msgs::msg::PoseStamped     (tracker pose inputs in world frame)
 * 
 * @author Daan Krijnen
 * @date September 10, 2026
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

constexpr double PI = 3.14159265359;

class DlsReducedController : public rclcpp::Node
{
    using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
    using TrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<FollowTrajectory>;
    using SwitchControllers = controller_manager_msgs::srv::SwitchController;
    using JointVector = Eigen::Matrix<double, 6, 1>;
    using JointMatrix = Eigen::Matrix<double, 6, 6>;

public:
    DlsReducedController(const rclcpp::NodeOptions & node_options)
    : Node("dls_reduced_controller", node_options)
    {

        load_parameters();

        initialize_ik_solver();

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

    struct TrackingErrors
    {
        Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
        Eigen::Vector3d error_outside = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_error = Eigen::Vector3d::Zero();
        double distance = 0.0;
    };

    rclcpp::Client<SwitchControllers>:: SharedPtr controller_switch_client_;

    rclcpp_action::Client<FollowTrajectory>::SharedPtr trajectory_client_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;

    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    OsqpEigen::Solver ik_solver_;

    moveit::core::RobotModelConstPtr robot_model_;

    const moveit::core::JointModelGroup * joint_model_group_;

    std::chrono::steady_clock::time_point joint_state_received_at_;
    std::chrono::steady_clock::time_point last_velocity_time_;
    std::chrono::steady_clock::time_point tracker_pose_received_at_;
    std::chrono::steady_clock::time_point previous_command_time_;
    std::chrono::steady_clock::time_point previous_joint_velocity_time_;

    sensor_msgs::msg::JointState latest_joint_state_;

    std::mutex joint_state_mutex_;
    std::mutex tracker_pose_mutex_;
    std::mutex velocity_mutex_;

    std::shared_ptr<moveit::core::RobotState> robot_state_;
    
    std::string tracker_frame_;
    std::string end_effector_name_;

    geometry_msgs::msg::PoseStamped latest_tracker_pose_;
    
    Eigen::Vector3d robot_near_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d tracker_reference_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d robot_reference_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond tracker_reference_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond robot_reference_orientation_ = Eigen::Quaterniond::Identity();

    double vx_ = 0.0;
    double vy_ = 0.0;
    double vz_ = 0.0;
    double wx_ = 0.0;
    double wy_ = 0.0;
    double wz_ = 0.0;

    double position_gain_;
    double orientation_gain_;
    double deadzone_translation_error_;
    double deadzone_orientation_error_;
    double deadzone_velocity_;
    double deadzone_twist_;
    double DLS_damping_;
    double braking_delay_;
    double stopping_margin_;
    double velocity_damping_gain_;
    double position_prio_distance_;
    double position_prio_max_mult_;
    double translation_scale_;
    double timeout_age_;
    double CONTROL_DT_;
    double position_priority_weight_;
    double velocity_priority_weight_;

    JointVector previous_commanded_velocity_ = JointVector::Zero();
    
    std::unordered_map<std::string, double> previous_joint_velocities_;
    std::unordered_map<std::string, double> latest_joint_accelerations_;

    bool velocity_limiting_;
    bool acceleration_limiting_;
    bool DEBUG_;
    
    bool ik_solver_initialized_ = false;
    bool have_previous_joint_velocity_sample_ = false;
    bool velocity_limiter_initialized_ = false;
    bool feedback_reference_set_ = false;
    bool received_tracker_pose_ = false;
    bool received_velocity_ = false;
    bool received_joint_state_ = false;

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

        if (!std::isfinite(deadzone_translation_error_) || deadzone_translation_error_ < 0.0)
        {
            throw std::runtime_error("deadzone translation error must be finite and larger or equal to zero");
        }

        deadzone_orientation_error_ = declare_parameter<double>("deadzone_orientation_error", 0.0);

        if (!std::isfinite(deadzone_orientation_error_) || deadzone_orientation_error_ < 0.0)
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

        if (max_velocity.size() != max_joint_velocity_.size())
        {
            throw std::runtime_error("max_velocity must have six values");
        }

        const auto max_acceleration = declare_parameter<std::vector<double>>("max_acceleration", std::vector<double>(6, 30.0));


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

        braking_delay_ = declare_parameter<double>("braking_delay", 0.1);

        if (!std::isfinite(braking_delay_) || braking_delay_ <= 0.0)
        {
            throw std::runtime_error("braking delay must be positive and finite");
        }

        stopping_margin_ = declare_parameter<double>("stopping_margin",0.01);

        if (!std::isfinite(stopping_margin_) || stopping_margin_ <= 0.0)
        {
            throw std::runtime_error("stopping margin must be positive and finite");
        }

        velocity_damping_gain_ = declare_parameter<double>("brake_damping_gain", 1.0);

        if (!std::isfinite(velocity_damping_gain_) || velocity_damping_gain_ <= 0.0)
        {
            throw std::runtime_error("brake damping gain must be positive and finite");
        }

        velocity_priority_weight_ = declare_parameter<double>("velocity_weight", 1.0);

        if (!std::isfinite(velocity_priority_weight_) || velocity_priority_weight_ <= 0.0)
        {
            throw std::runtime_error("velocity priority must be positive and finite");
        }

        position_priority_weight_ = declare_parameter<double>("position_weight", 1.0);

        if (!std::isfinite(position_priority_weight_) || position_priority_weight_ <= 0.0)
        {
            throw std::runtime_error("position priority must be positive and finite");
        }

        position_prio_distance_ = declare_parameter<double>("position_priority_distance", 0.05);

        if (!std::isfinite(position_prio_distance_) || position_prio_distance_ <= 0.0)
        {
            throw std::runtime_error("position priority distance must be positive and finite");
        }

        position_prio_max_mult_ = declare_parameter<double>("position_priority_max_multiplier", 10.0);

        if (!std::isfinite(position_prio_max_mult_) || position_prio_max_mult_ <= 0.0)
        {
            throw std::runtime_error("position priority max multiplier must be positive and finite");
        }

        end_effector_name_ = declare_parameter<std::string>("end_effector_name", "Unknown");

        if (end_effector_name_.empty())
        {
            throw std::runtime_error("End effector name cannot be empty");
        }

        std::cerr << "=============================================" << std::endl;
        std::cerr << "              Parameters Loaded" << std::endl;
        std::cerr << "=============================================" << std::endl;
        std::cerr << "   controller timer:           " << CONTROL_DT_ << " [s]" << std::endl;
        std::cerr << "   shows logs:                 " << (DEBUG_ ? "True":"False") << std::endl;
        std::cerr << "   translation scale:          " << translation_scale_ << std::endl;
        std::cerr << "   end effector name:          " << end_effector_name_ << std::endl;
        std::cerr << "   robot start position:    x: " << robot_near_.x() << " [m]" << std::endl;
        std::cerr << "                            y: " << robot_near_.y() << " [m]" << std::endl;
        std::cerr << "                            z: " << robot_near_.z() << " [m]" << std::endl;
        std::cerr << "   deadzone velocity:          " << deadzone_velocity_ << " [m/s]" << std::endl;
        std::cerr << "   deadzone twist:             " << deadzone_twist_ << " [rad/s]" << std::endl;
        std::cerr << "   tracker frame:              " << tracker_frame_ << std::endl;
        std::cerr << "   timeout age:                " << timeout_age_ << " [s]" << std::endl;
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
        std::cerr << "   braking delay:              " << braking_delay_ << " [s]" << std::endl;
        std::cerr << "   stopping margin:            " << stopping_margin_ << " [m]" << std::endl;
        std::cerr << "   brake damping gain:         " << velocity_damping_gain_ << std::endl;
        std::cerr << "   velocity priority:          " << velocity_priority_weight_ << std::endl;
        std::cerr << "   position priority:          " << position_priority_weight_ << std::endl;
        std::cerr << "   position priority distance: " << position_prio_distance_ << " [m]" << std::endl;
        std::cerr << "   priority max multiplier:    " << position_prio_max_mult_ << std::endl;
        std::cerr << "=============================================" << std::endl;
    }

    void initialize_ik_solver()
    {
        Eigen::SparseMatrix<double> hessian(6, 6);

        std::vector<Eigen::Triplet<double>> hessian_triplets;
        hessian_triplets.reserve(21);

        for (int col = 0; col < 6; ++col)
        {
            for (int row = 0; row <= col; ++row)
            {
                // Explicitly retain every upper-triangular entry,
                // including entries that initially have value zero.
                hessian_triplets.emplace_back(row, col, row == col ? 1.0 : 0.0);
            }
        }

        hessian.setFromTriplets(
            hessian_triplets.begin(),
            hessian_triplets.end());

        Eigen::SparseMatrix<double> constraints(6, 6);
        constraints.setIdentity();

        hessian.makeCompressed();
        constraints.makeCompressed();

        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd lower_bound = Eigen::VectorXd::Constant(6, -1.0);
        Eigen::VectorXd upper_bound = Eigen::VectorXd::Constant(6, 1.0);

        auto & data = ik_solver_.data();

        ik_solver_.settings()->setVerbosity(false);
        ik_solver_.settings()->setWarmStart(true);

        data->setNumberOfVariables(6);
        data->setNumberOfConstraints(6);

        RCLCPP_INFO(get_logger(),
            "Initializing OSQP solver...");

        if (!data->setHessianMatrix(hessian))
        {
            throw std::runtime_error("OSQP: setHessianMatrix failed");
        }

        if (!data->setGradient(gradient))
        {
            throw std::runtime_error("OSQP: setGradient failed");
        }

        if (!data->setLinearConstraintsMatrix(constraints))
        {
            throw std::runtime_error("OSQP: setLinearConstraintsMatrix failed");
        }

        if (!data->setLowerBound(lower_bound))
        {
            throw std::runtime_error("OSQP: setLowerBound failed");
        }

        if (!data->setUpperBound(upper_bound))
        {
            throw std::runtime_error("OSQP setUpperBound failed");
        }


        if (!ik_solver_.initSolver())
        {
            throw std::runtime_error(
                "Failed to initialize constrained IK solver");

            return;
        }

        RCLCPP_INFO(get_logger(),
            "OSQP solver initialized successfully.");

        ik_solver_initialized_ = true;

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

        const auto * end_effector = robot_model->getLinkModel(end_effector_name_);

        if (end_effector == nullptr)
        {
            throw std::runtime_error(end_effector_name_ + " not found");
        }

        Eigen::Isometry3d target_pose = goal_state.getGlobalLinkTransform(end_effector);

        target_pose.translation() = robot_near_;

        if (!goal_state.setFromIK(
            joint_model_group_,
            target_pose,
            end_effector_name_,
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
            RCLCPP_INFO(get_logger(),
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

        const auto * ee = robot_model_->getLinkModel(end_effector_name_);
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

    void get_tracker_twist(
        Eigen::Vector3d & linear_velocity,
        Eigen::Vector3d & angular_velocity)
    {
        linear_velocity.setZero();
        angular_velocity.setZero();

        std::lock_guard<std::mutex> lock(velocity_mutex_);

        const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_velocity_time_).count();

        if (!received_velocity_ || age > timeout_age_)
        {
            return;
        }

        linear_velocity = Eigen::Vector3d(vx_, vy_, vz_);
        angular_velocity = Eigen::Vector3d(wx_, wy_, wz_);

        for (Eigen::Index i = 0; i < 3; ++i)
        {
            if (std::abs(linear_velocity[i]) < deadzone_velocity_)
            {
                linear_velocity[i] = 0.0;
            }

            if (std::abs(angular_velocity[i]) < deadzone_twist_)
            {
                angular_velocity[i] = 0.0;
            }
        }
    }

    bool get_ordered_joint_velocities(
        const sensor_msgs::msg::JointState & joint_state,
        JointVector & velocities)
    {
        const auto & joint_names = joint_model_group_->getActiveJointModelNames();

        if (joint_names.size() != 6)
        {
            RCLCPP_ERROR(get_logger(),
                "Expected 6 active joints, found %zu",
                joint_names.size());
            return false;
        }

        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            const auto name_it = std::find(
                joint_state.name.begin(),
                joint_state.name.end(),
                joint_names[i]);

            if (name_it == joint_state.name.end())
            {
                RCLCPP_WARN(get_logger(),
                    "Missing joint state for %s",
                    joint_names[i].c_str());
                return false;
            }

            const auto state_index = static_cast<std::size_t>(std::distance(joint_state.name.begin(), name_it));

            if (state_index >= joint_state.velocity.size() || !std::isfinite(joint_state.velocity[state_index]))
            {
                RCLCPP_WARN(get_logger(),
                    "Missing or invalid joint velocity for %s",
                    joint_names[i].c_str());

                return false;
            }

            velocities[static_cast<Eigen::Index>(i)] = joint_state.velocity[state_index];
        }

        return true;
    }

    TrackingErrors calculate_tracking_errors(
        const Eigen::Vector3d & tracker_position,
        const Eigen::Vector3d & robot_position,
        const Eigen::Quaterniond & tracker_orientation,
        const Eigen::Quaterniond & robot_orientation) const
    {
        TrackingErrors errors;

        const Eigen::Vector3d tracker_delta = tracker_position - tracker_reference_position_;

        const Eigen::Vector3d target_position = robot_reference_position_ + translation_scale_ * tracker_delta;

        errors.position_error = target_position - robot_position;
        errors.distance = errors.position_error.norm();

        if (errors.distance > deadzone_translation_error_)
        {
            errors.error_outside = errors.position_error * ((errors.distance - deadzone_translation_error_) / errors.distance);
        }

        Eigen::Quaterniond tracker_orientation_delta = tracker_orientation * tracker_reference_orientation_.conjugate();
        tracker_orientation_delta.normalize();

        Eigen::Quaterniond target_orientation = tracker_orientation_delta * robot_reference_orientation_;
        target_orientation.normalize();

        Eigen::Quaterniond orientation_error = target_orientation * robot_orientation.conjugate();
        orientation_error.normalize();

        if (orientation_error.w() < 0.0)
        {
            orientation_error.coeffs() *= -1;
        }

        const double sin_half_angle = orientation_error.vec().norm();

        if (sin_half_angle > 0.0)
        {
            const double angle = 2.0 * std::atan2(sin_half_angle, orientation_error.w());
        
            if (angle > deadzone_orientation_error_)
            {
                errors.angular_error = ((angle - deadzone_orientation_error_) / sin_half_angle) * orientation_error.vec();
            }
        }

        return errors;
    }

    bool get_current_jacobian(
        const std::vector<double> & joint_positions,
        JointMatrix & jacobian)
    {
        robot_state_->setJointGroupPositions(
            joint_model_group_,
            joint_positions);
        robot_state_->update();

        const auto * end_effector = robot_model_->getLinkModel(end_effector_name_);

        if (end_effector == nullptr)
        {
            RCLCPP_ERROR(get_logger(), 
                "%s was not found",
                end_effector_name_.c_str());

            return false;
        }

        Eigen::MatrixXd dynamic_jacobian;
        robot_state_->getJacobian(
            joint_model_group_,
            end_effector,
            Eigen::Vector3d::Zero(),
            dynamic_jacobian);

        if (dynamic_jacobian.rows() != 6 ||
            dynamic_jacobian.cols() != 6 ||
            !dynamic_jacobian.allFinite())
        {
            RCLCPP_ERROR(get_logger(), "Jacobian is invalid or not 6x6");
            return false;
        }

        jacobian = dynamic_jacobian;
        return true;
    }

    bool calculate_priority_weights(
        const double distance,
        double & position_weight,
        double & velocity_weight) const
    {
        const double transition_distance = std::max(position_prio_distance_ - deadzone_translation_error_, 0.0001);

        const double gap_ratio = std::clamp((distance - deadzone_translation_error_) / transition_distance, 0.0, 1.0);

        position_weight = position_priority_weight_ * (1.0 + (position_prio_max_mult_ - 1.0) * gap_ratio);
        velocity_weight = velocity_priority_weight_;

        return std::isfinite(position_weight) &&
            std::isfinite(velocity_weight) &&
            position_weight > 0.0 &&
            velocity_weight > 0.0;
    }

    bool apply_directional_braking_limit(
        const Eigen::Vector3d & position_error,
        const double distance,
        const JointMatrix & jacobian,
        const Eigen::Vector3d relative_velocity,
        const Eigen::Vector3d & feedforward_velocity,
        const double position_weight,
        const double total_weight,
        JointVector & position_twist,
        Eigen::Vector3d & combined_linear_velocity)
    {
        if (distance > 0.0)
        {
            const Eigen::Vector3d direction = position_error / distance;

            double directional_brake_acceleration = 0.0;

            for (Eigen::Index i = 0; i < 6; ++i)
            {
                const double directional_contribution = direction.dot(jacobian.block<3, 1>(0, i));

                directional_brake_acceleration += std::abs(directional_contribution) * max_joint_acceleration_[static_cast<std::size_t>(i)];
            }

            if (!std::isfinite(directional_brake_acceleration) ||
                !combined_linear_velocity.allFinite() ||
                directional_brake_acceleration < 0.0)
            {
                return false;
            }

            const double closing_speed = relative_velocity.dot(direction);

            const double reaction_distance = std::max(closing_speed, 0.0) * braking_delay_;

            const double available_distance = std::max(distance - stopping_margin_ - reaction_distance, 0.0);

            const double allowed_closing_speed = std::sqrt(2.0 * directional_brake_acceleration * available_distance);

            const double commanded_closing_speed = (combined_linear_velocity - feedforward_velocity).dot(direction);

            if (commanded_closing_speed > allowed_closing_speed)
            {
                const Eigen::Vector3d braked_linear_velocity = combined_linear_velocity - (commanded_closing_speed - allowed_closing_speed) * direction;

                position_twist.head<3>() += (total_weight / position_weight) * (braked_linear_velocity - combined_linear_velocity);

                combined_linear_velocity = braked_linear_velocity;
            }
        }

        return true;
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

        Eigen::Vector3d tracker_velocity;
        Eigen::Vector3d tracker_angular_velocity;
        get_tracker_twist(tracker_velocity, tracker_angular_velocity);

        const TrackingErrors tracking_errors = calculate_tracking_errors(
            tracker_position,
            robot_position,
            tracker_orientation,
            robot_orientation);

        const Eigen::Vector3d & position_error = tracking_errors.position_error;
        const Eigen::Vector3d & error_outside = tracking_errors.error_outside;
        const Eigen::Vector3d angular_error = tracking_errors.angular_error;
        const double distance = tracking_errors.distance;
        
        const Eigen::Vector3d angular_correction = orientation_gain_ * angular_error;

        JointMatrix jacobian = JointMatrix::Zero();

        if (!get_current_jacobian(current_positions, jacobian))
        {
            publish_zero_velocity();
            return;
        }

        JointVector measured_joint_velocity = JointVector::Zero();

        if (!get_ordered_joint_velocities(joint_state, measured_joint_velocity))
        {
            publish_zero_velocity();
            return;
        }

        const Eigen::Vector3d robot_ee_velocity = jacobian.topRows<3>() * measured_joint_velocity;

        const Eigen::Vector3d feedforward_velocity = translation_scale_ * tracker_velocity;

        const Eigen::Vector3d relative_velocity = robot_ee_velocity - feedforward_velocity;

        Eigen::Vector3d position_correction = position_gain_ * error_outside - velocity_damping_gain_ * relative_velocity;

        double position_weight_effective = 0.0;
        double velocity_weight_effective = 0.0;

        if (!calculate_priority_weights(
            distance,
            position_weight_effective,
            velocity_weight_effective))
        {
            publish_zero_velocity();
            return;
        }

        JointVector position_twist = JointVector::Zero();
        JointVector feedforward_twist = JointVector::Zero();


        position_twist.head<3>() = position_correction;
        feedforward_twist.head<3>() = feedforward_velocity;

        position_twist.tail<3>() = angular_correction;
        feedforward_twist.tail<3>() = tracker_angular_velocity;


        const double total_weight = position_weight_effective + velocity_weight_effective;

        if (!std::isfinite(total_weight) || total_weight <= 0.0)
        {
            publish_zero_velocity();
            return;
        }

        Eigen::Vector3d combined_linear_velocity =
            (position_weight_effective * position_twist.head<3>() +
            velocity_weight_effective * feedforward_twist.head<3>())
            / total_weight;


        if (!apply_directional_braking_limit(
            position_error,
            distance,
            jacobian,
            relative_velocity,
            feedforward_velocity,
            position_weight_effective,
            total_weight,
            position_twist,
            combined_linear_velocity))
        {
            publish_zero_velocity();
            return;
        }

        if (!feedforward_twist.allFinite() ||
            !position_twist.allFinite())
        {
            publish_zero_velocity();
            return;
        }

        const auto now = std::chrono::steady_clock::now();

        double dt = CONTROL_DT_;

        if (velocity_limiter_initialized_)
        {
            dt = std::chrono::duration<double>(now - previous_command_time_).count();
        }
        else
        {
            previous_commanded_velocity_.setZero();
            velocity_limiter_initialized_ = true;
        }
        
        previous_command_time_ = now;

        if (!std::isfinite(dt) || dt <= 0.0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Invalid or excessively long control-loop interval: %.4f s",
                dt);

            publish_zero_velocity();
            previous_commanded_velocity_.setZero();
            return;
        }


        JointVector commanded_joint_velocity = JointVector::Zero();


        const bool qp_success = solve_constrained_ik(
            jacobian,
            position_twist,
            feedforward_twist,
            dt,
            previous_commanded_velocity_,
            position_weight_effective,
            velocity_weight_effective,
            commanded_joint_velocity);

        if (!qp_success || !commanded_joint_velocity.allFinite())
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Constrained IK failed; commanding zero velocity");

            publish_zero_velocity();

            previous_commanded_velocity_.setZero();
            return;
        }

        previous_commanded_velocity_ = commanded_joint_velocity;



        if (DEBUG_)
        {

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
            "Final v[m/s]=[%.3f %.3f %.3f] w[rad/s]=[%.3f %.3f %.3f] "
            "Distance=%.4f",
            tracker_position.x(), tracker_position.y(), tracker_position.z(),
            tracker_orientation.x(), tracker_orientation.y(), tracker_orientation.z(), tracker_orientation.w(),
            robot_position.x(), robot_position.y(), robot_position.z(),
            robot_orientation.x(), robot_orientation.y(), robot_orientation.z(), robot_orientation.w(),
            position_error.x(), position_error.y(), position_error.z(),
            angular_error.x(), angular_error.y(), angular_error.z(),
            feedforward_velocity.x(), feedforward_velocity.y(), feedforward_velocity.z(),
            tracker_angular_velocity.x(), tracker_angular_velocity.y(), tracker_angular_velocity.z(),
            position_correction.x(), position_correction.y(), position_correction.z(),
            angular_correction.x(), angular_correction.y(), angular_correction.z(),
            combined_linear_velocity.x(), combined_linear_velocity.y(), combined_linear_velocity.z(),
            (tracker_angular_velocity + angular_correction).x(),
            (tracker_angular_velocity + angular_correction).y(),
            (tracker_angular_velocity + angular_correction).z(),
            distance);

            
            
            const auto & joint_names = joint_model_group_->getActiveJointModelNames();

            std::ostringstream velocity_log;
            velocity_log << std::fixed << std::setprecision(6);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
                if (i > 0)
                {
                    velocity_log << " | ";
                }

                velocity_log << joint_names[i]
                                << " requested=" << commanded_joint_velocity[static_cast<Eigen::Index>(i)]
                                << " actual=" << measured_joint_velocity[static_cast<Eigen::Index>(i)];

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

        publish_joint_velocity_command(commanded_joint_velocity);
    }


    bool solve_constrained_ik(
        const JointMatrix & jacobian,
        const JointVector & position_twist,
        const JointVector & feedforward_twist,
        const double dt,
        const JointVector & previous_command,
        const double position_weight,
        const double velocity_weight,
        JointVector & solution)
    {
        constexpr int N = 6;

        if (!ik_solver_initialized_ ||
            !jacobian.allFinite() ||
            !position_twist.allFinite() ||
            !feedforward_twist.allFinite() ||
            !previous_command.allFinite() ||
            !std::isfinite(position_weight) ||
            !std::isfinite(velocity_weight) ||
            position_weight <= 0.0 ||
            velocity_weight < 0.0 ||
            !std::isfinite(dt))
        {
            return false;
        }

        const double wp = position_weight;
        const double wv = velocity_weight;

        // Damped least-squares objective:
        // min 0.5*wp*||J*qdot - position_twist||^2
        //   + 0.5*wv*||J*qdot - feedforward_twist||^2
        //   + 0.5*lambda^2*||qdot||^2
        JointMatrix hessian_dense = (wp + wv) * (jacobian.transpose() * jacobian);

        hessian_dense.diagonal().array() += DLS_damping_ * DLS_damping_;

        const JointVector gradient = -jacobian.transpose() * (wp * position_twist + wv * feedforward_twist);

        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(21);

        for (int col = 0; col < N; ++col)
        {
            for (int row = 0; row <= col; ++row)
            {
                const double value = hessian_dense(row, col);

                if (!std::isfinite(value))
                {
                    return false;
                }

                triplets.emplace_back(row, col, value);
            }
        }

        Eigen::SparseMatrix<double> hessian_upper(N, N);
        hessian_upper.setFromTriplets(
            triplets.begin(),
            triplets.end());

        hessian_upper.makeCompressed();

        JointVector lower(N);
        JointVector upper(N);

        for (int i = 0; i < N; ++i)
        {
            const std::size_t index = static_cast<std::size_t>(i);

            //joint velocity constraints
            double lower_i = velocity_limiting_ ? -max_joint_velocity_[index] : -std::numeric_limits<double>::infinity();

            double upper_i = velocity_limiting_ ? max_joint_velocity_[index] : std::numeric_limits<double>::infinity();

            if (acceleration_limiting_)
            {
                const double max_delta = max_joint_acceleration_[index] * dt;

                lower_i = std::max(lower_i, previous_command[i] - max_delta);

                upper_i = std::min(upper_i, previous_command[i] + max_delta);
            }

            if (lower_i > upper_i)
            {
                return false;
            }

            lower[i] = lower_i;
            upper[i] = upper_i;
        }

        //Update QP data without reinitializing solver.
        if (!ik_solver_.updateHessianMatrix(hessian_upper))
        {
            RCLCPP_ERROR(get_logger(), "OSQP updateHessianMatrix failed");
            return false;
        }
        if (!ik_solver_.updateGradient(gradient))
        {
            RCLCPP_ERROR(get_logger(), "OSQP updateGradient failed");
            return false;
        }
        if (!ik_solver_.updateBounds(lower, upper))
        {
            RCLCPP_ERROR(get_logger(), "OSQP updateBounds failed");
            return false;
        }

        if (ik_solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            return false;
        }

        const Eigen::VectorXd result = ik_solver_.getSolution();

        if (result.size() != N || !result.allFinite())
        {
            return false;
        }

        solution = result;
        return true;
    }


    void publish_zero_velocity()
    {

        previous_commanded_velocity_.setZero();
        previous_command_time_ = std::chrono::steady_clock::now();
        velocity_limiter_initialized_ = true;

        publish_joint_velocity_command(JointVector::Zero());
    }


    void publish_joint_velocity_command(const JointVector & velocities)
    {
        std_msgs::msg::Float64MultiArray command;
        command.data.resize(static_cast<std::size_t>(velocities.size()));

        for (Eigen::Index i = 0; i < velocities.size(); ++i)
        {
            command.data[static_cast<std::size_t>(i)] = velocities[i];
        }

        velocity_publisher_->publish(command);
    }
};

int main(int argc, char * argv[])
{
    std::cerr << "========== DLS EXECUTABLE STARTED ==========" << std::endl;

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    auto node = std::make_shared<DlsReducedController>(node_options);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node);
    
    std::cerr << "========== CALLING INITIALIZE_SYSTEM ==========" << std::endl;

    node->initialize_system();

    std::cerr << "========== INITIALIZE_SYSTEM FINISHED ==========" << std::endl;

    executor.spin();

    node->stop_motion();

    rclcpp::shutdown();
    return 0;
}