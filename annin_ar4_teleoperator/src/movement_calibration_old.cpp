/**
 * @file movement_calibration_old.cpp
 * @brief A old verion of script to use for calibration with the AR4 arm and libsurvive velocity input.
 * 
 *  It sets the base static frame for the system to refer to.
 * 
 *  Calibration procedure:
 *      1. Record starting position for X+
 *      2. Move tracker in desired X+ direction
 *      3. record end position for X+
 * 
 *      4. record stating position for Y+
 *      5. move tracker in desired Y+ direction
 *      6. record end position for Y+
 * 
 *      7. Calculate Z+ direction using right-hand rule
 * 
 * -------------------------
 * Publishing Topics
 * 
 * -------------------------
 * Subscribed Topics
 * 
 *  velocity_subscription_      /libsurvive/velocity       geometry_msgs::msg::TwistStamped        (used to get reference frame)
 * 
 * @author Daan Krijnen
 * @date September 22, 2026
 */

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <filesystem>

class MovementCalibration : public rclcpp::Node
{
public:

    MovementCalibration(const rclcpp::NodeOptions & node_options)
        : Node("movement_calibration", node_options)
    {
        target_frame_ = "teleop_world";
        survive_frame_ = "steamvr_world";

        velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/tracker/velocity", 10, std::bind(&MovementCalibration::velocity_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
            this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(),
        "======================================");
        RCLCPP_INFO(get_logger(),
        "          Teleop Calibration");
        RCLCPP_INFO(get_logger(),
        "======================================");
        RCLCPP_INFO(get_logger(),
        "Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(get_logger(),
        "Survive frame: %s", survive_frame_.c_str());
        RCLCPP_INFO(get_logger(),
        "Coordinate convention:");
        RCLCPP_INFO(get_logger(),
        "   X+ = RIGHT");
        RCLCPP_INFO(get_logger(),
        "   Y+ = FORWARD");
        RCLCPP_INFO(get_logger(),
        "   Z+ = UP");
        RCLCPP_INFO(get_logger(),
        " ");
        RCLCPP_INFO(get_logger(),
        "Searching for Tracker...");
        RCLCPP_INFO(get_logger(),
        "======================================");

        calibration_thread_ = std::thread(&MovementCalibration::run_calibration, this);
    }

    ~MovementCalibration()
    {
        if (calibration_thread_.joinable())
        {
            calibration_thread_.join();
        }
    }

private:
    std::string target_frame_;
    std::string survive_frame_;
    std::string tracker_frame_;

    std::mutex tracker_mutex_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
    bool received_velocity_ = false;

    std::thread calibration_thread_;

    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        const std::string source_frame = msg->header.frame_id;

        if (source_frame.empty())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Received /velocity with empty frame_id");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(tracker_mutex_);

            if (tracker_frame_.empty())
            {
                tracker_frame_ = source_frame;

                RCLCPP_INFO(get_logger(), "Tracker detected: %s",
                    tracker_frame_.c_str());
            }
        }

        received_velocity_ = true;
    }

    std::string get_tracker_frame()
    {
        std::lock_guard<std::mutex> lock(tracker_mutex_);

        return tracker_frame_;
    }

    Eigen::Vector3d get_tracker_position()
    {
        geometry_msgs::msg::TransformStamped transform;
        std::string tracker_frame = get_tracker_frame();

        if (tracker_frame.empty())
        {
            throw std::runtime_error("Tracker frame has not been detected yet.");
        }

        try
        {
            transform = tf_buffer_->lookupTransform(survive_frame_,
                                                    tracker_frame,
                                                    rclcpp::Time(0),
                                                    rclcpp::Duration::from_seconds(0.1));
        }
        catch(const tf2::TransformException & ex)
        {
            RCLCPP_ERROR(get_logger(),
            "Could not get tracker transform: %s", ex.what());

            throw;
        }

        return Eigen::Vector3d( transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z);
        
    }

    void wait_for_enter(const std::string & message)
    {
        std::cout << std::endl;
        std::cout << message << std::endl;
        std::cout << "Press ENTER when ready..." << std::endl;

        std::string input;

        std::getline(std::cin, input);
    }

    void wait_for_tracker()
    {
        RCLCPP_INFO(get_logger(), "waiting for tracker frame from /velocity topic...");

        while (rclcpp::ok())
        {
            if (received_velocity_)
            {
                std::string tracker_frame = get_tracker_frame();

                if (!tracker_frame.empty())
                {
                    RCLCPP_INFO(get_logger(), "Tracker Detected: %s",
                        tracker_frame.c_str());

                    return;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void run_calibration()
    {
        wait_for_tracker();

        if (!rclcpp::ok())
        {
            return;
        }

        std::string tracker_frame = get_tracker_frame();

        std::cout << std::endl;
        std::cout << "It is adviced to use a flat surface for" << std::endl;
        std::cout << "calibrating to make sure all directions" << std::endl;
        std::cout << "do not interfere with one another." << std::endl;
        std::cout << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "Calibrate X+" <<std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "Tracker Detected" << std::endl;
        std::cout << "   " << tracker_frame << std::endl;
        std::cout << std::endl;
        std::cout << "Place tracker at starting position." << std::endl;
        
        wait_for_enter("Keep the tracker still.");

        Eigen::Vector3d x_start = get_tracker_position();

        std::cout << "Recorded X start:" << std::endl;
        
        print_vector(x_start);

        std::cout << std::endl;

        std::cout << "Move tracker physically in desired" << std::endl;
        std::cout << "positive X direction (right)" << std::endl;
        std::cout << "Move tracker approx 20 to 40 cm" << std::endl;

        wait_for_enter("keep tracker still at end position");

        Eigen::Vector3d x_end = get_tracker_position();

        std::cout << "Recorded X end:" << std::endl;

        print_vector(x_end);

        Eigen::Vector3d x_direction = x_end - x_start;

        if (x_direction.norm() < 0.05)
        {
            RCLCPP_ERROR(get_logger(),
            "X movement was too small to detect!");
            return;
        }
        x_direction.normalize();


        std::cout << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "Calibrate Y+" << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "Return tracker to convenient position" << std::endl;

        wait_for_enter("Keep tracker still at start position.");

        Eigen::Vector3d y_start = get_tracker_position();

        std::cout << "Recorded Y start:" << std::endl;

        print_vector(y_start);

        std::cout << "Move tracker physically in desired" << std::endl;
        std::cout << "positive Y direction (forwards)" << std::endl;
        std::cout << "Move tracker approx 20 to 40 cm" << std::endl;

        wait_for_enter("Keep tracker still at end position");

        Eigen::Vector3d y_end = get_tracker_position();

        std::cout << "Recorded Y end" << std::endl;

        print_vector(y_end);

        Eigen::Vector3d y_raw = y_end - y_start;

        if (y_raw.norm() < 0.05)
        {
            RCLCPP_ERROR(get_logger(),
            "Y movement was too small to detect!");
            return;
        }
        y_raw.normalize();


        Eigen::Vector3d y_direction = y_raw - x_direction * x_direction.dot(y_raw);

        if (y_direction.norm() < 0.01)
        {
            RCLCPP_ERROR(get_logger(),
            "X and Y directions are too similar.");
            return;
        }

        y_direction.normalize();


        Eigen::Vector3d z_direction = x_direction.cross(y_direction);
        z_direction.normalize();

        y_direction = z_direction.cross(x_direction);
        y_direction.normalize();


        std::cout << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "      CALIBRATED COORDINATE SYSTEM" << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "X+ = Right:" << std::endl;

        print_vector(x_direction);

        std::cout << std::endl;
        std::cout << "Y+ = Forwards:" << std::endl;
        
        print_vector(y_direction);

        std::cout << std::endl;
        std::cout << "Z+ = Up:" << std::endl;

        print_vector(z_direction);


        Eigen::Matrix3d R_survive_from_world;
        
        R_survive_from_world.col(0) = x_direction;
        R_survive_from_world.col(1) = y_direction;
        R_survive_from_world.col(2) = z_direction;


        Eigen::Matrix3d R_world_from_survive = R_survive_from_world.transpose();

        Eigen::Quaterniond quaternion(R_world_from_survive);
        quaternion.normalize();


        std::cout << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "          Calibration Quaternion" << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "X = " << quaternion.x() << std::endl;
        std::cout << "Y = " << quaternion.y() << std::endl;
        std::cout << "Z = " << quaternion.z() << std::endl;
        std::cout << "W = " << quaternion.w() << std::endl;


        geometry_msgs::msg::TransformStamped static_transform;

        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = target_frame_;
        static_transform.child_frame_id = survive_frame_;


        //no translation calibration yet
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;

        static_transform.transform.rotation.x = quaternion.x();
        static_transform.transform.rotation.y = quaternion.y();
        static_transform.transform.rotation.z = quaternion.z();
        static_transform.transform.rotation.w = quaternion.w();

        static_broadcaster_->sendTransform(static_transform);


        std::cout << "=============================================" << std::endl;
        std::cout << "           Calibration Complete" << std::endl;
        std::cout << "=============================================" << std::endl;
        std::cout << "Tracker Used:" << std::endl;
        std::cout << "  " << tracker_frame << std::endl;
        std::cout << std::endl;

        std::cout << "Published:" << std::endl;
        std::cout << "  " << target_frame_ << " -> " << survive_frame_ << std::endl;
        std::cout << std::endl;

        std::cout << "Coordinate Conversion:" << std::endl;
        std::cout << "  X+ = Right" << std::endl;
        std::cout << "  Y+ = Forward" << std::endl;
        std::cout << "  Z+ = Up" << std::endl;
        std::cout << "=============================================" << std::endl;

        save_calibration(x_direction, y_direction, z_direction, quaternion);
    }

    void print_vector(const Eigen::Vector3d & vector)
    {
        std::cout   << "  [ " 
                    << vector.x() << ", " 
                    << vector.y() << ", " 
                    << vector.z() << " ]" 
                    << std::endl;
    }

    void save_calibration(  const Eigen::Vector3d & x_direction,
                            const Eigen::Vector3d & y_direction,
                            const Eigen::Vector3d & z_direction,
                            const Eigen::Quaterniond & quaternion)
    {
        const char * home_directory = std::getenv("HOME");

        if (home_directory == nullptr)
        {
            RCLCPP_ERROR(get_logger(),"Could not determine HOME directory!");

            return;
        }

        const std::filesystem::path config_directory = std::filesystem::path(home_directory) / ".config" / "annin_ar4_teleoperator";
        const std::filesystem::path calibration_file = config_directory / "calibration.yaml";

        try
        {
            std::filesystem::create_directories(config_directory);
        }
        catch(const std::filesystem::filesystem_error & ex)
        {
            RCLCPP_ERROR(get_logger(), "Could not create calibration directory: %s", ex.what());

            return;
        }

        YAML::Emitter out;

        out << YAML::BeginMap;

        out << YAML::Key << "calibration";
        out << YAML::BeginMap;

        //Frame names
        out << YAML::Key << "target_frame"
            << YAML::Value << target_frame_;
        out << YAML::Key << "survive_frame"
            << YAML::Value << survive_frame_;
        out << YAML::Key << "tracker_frame"
            << YAML::Value << get_tracker_frame();

        //Translation
        out << YAML::Key << "translation";
        out << YAML::BeginMap;

        out << YAML::Key << "x"
            << YAML::Value << 0.0;
        out << YAML::Key << "y"
            << YAML::Value << 0.0;
        out << YAML::Key << "z"
            << YAML::Value << 0.0;
        
        out << YAML::EndMap;

        //rotation
        out << YAML::Key << "rotation";
        out << YAML::BeginMap;

        out << YAML::Key << "x"
            << YAML::Value << quaternion.x();
        out << YAML::Key << "y"
            << YAML::Value << quaternion.y();
        out << YAML::Key << "z"
            << YAML::Value << quaternion.z();
        out << YAML::Key << "w"
            << YAML::Value << quaternion.w();

        out << YAML::EndMap;

        //callibrated axes
        auto write_vector = [&](const std::string & name,
                                const Eigen::Vector3d & vector)
        {
            out << YAML::Key << name;
            out << YAML::BeginMap;

            out << YAML::Key << "x"
                << YAML::Value << vector.x();
            out << YAML::Key << "y"
                << YAML::Value << vector.y();
            out << YAML::Key << "z"
                << YAML::Value << vector.z();

            out << YAML::EndMap;
        };
        
        out << YAML::Key << "axes";
        out << YAML::BeginMap;

        write_vector("x", x_direction);
        write_vector("y", y_direction);
        write_vector("z", z_direction);

        out << YAML::EndMap;

        //coordinate conversion
        out << YAML::Key << "coordinate_conversion";
        out << YAML::BeginMap;

        out << YAML::Key << "x"
            << YAML::Value << "right";
        out << YAML::Key << "y"
            << YAML::Value << "forward";
        out << YAML::Key << "z"
            << YAML::Value << "up";

        out << YAML::EndMap;

        out << YAML::EndMap;
        out << YAML::EndMap;

        try
        {
            std::ofstream file(calibration_file);

            if (!file.is_open())
            {
                RCLCPP_ERROR(get_logger(), "Could not open calibration file: %s",
                    calibration_file.c_str());
                
                return;
            }

            file << out.c_str();
            file.close();
        }
        catch(const std::exception & ex)
        {
            RCLCPP_ERROR(get_logger(),"Could not save calibration: %s", ex.what());

            return;
        }

        RCLCPP_INFO(get_logger(),"Calibration saved to: %s", calibration_file.c_str());
        
    }
};



int main (int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MovementCalibration>(node_options);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}