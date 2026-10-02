/**
 * @file movement_calibration.cpp
 * @brief A script to use for calibration with the AR4 arm and libsurvive velocity input.
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
 *  pose_subscription_      /libsurvive/velocity       geometry_msgs::msg::TwistStamped        (used to get reference frame)
 * 
 * @author Daan Krijnen
 * @date October 01, 2026
 */

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.hpp>

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
    source_frame_ = "steamvr_world";

    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tracker/pose", 10, std::bind(&MovementCalibration::pose_callback, this, std::placeholders::_1));
    
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
    "Source frame: %s", source_frame_.c_str());
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
  std::string source_frame_;

  std::mutex tracker_mutex_;

  Eigen::Vector3d latest_tracker_position_ = Eigen::Vector3d::Zero();
  bool received_pose_ = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::thread calibration_thread_;

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(tracker_mutex_);

    latest_tracker_position_ = Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z
    );

    if (!msg->header.frame_id.empty())
    {
      source_frame_ = msg->header.frame_id;
    }
    received_pose_ = true;
  }

  Eigen::Vector3d get_tracker_position()
  {
    std::lock_guard<std::mutex> lock(tracker_mutex_);

    if (!received_pose_)
    {
      throw std::runtime_error("Tracker pose has not been received yet.");
    }

    return latest_tracker_position_;
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
    RCLCPP_INFO(get_logger(),"waiting for tracker pose...");

    while (rclcpp::ok())
    {
      {
        std::lock_guard<std::mutex> lock(tracker_mutex_);

        if (received_pose_)
        {
          RCLCPP_INFO(get_logger(), "Tracker detected!");
          RCLCPP_INFO(get_logger(), "Tracker frame: %s", source_frame_.c_str());

          RCLCPP_INFO(get_logger(), "Tracker position: [%.5f, %.5f, %.5f]",
            latest_tracker_position_.x(),
            latest_tracker_position_.y(),
            latest_tracker_position_.z());

          return;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void run_calibration()
  {
    wait_for_tracker();

    if(!rclcpp::ok())
    {
      return;
    }

    std::cout << std::endl;
    std::cout << "It is adviced to use a flat surface for" << std::endl;
    std::cout << "calibrating to make sure all directions" << std::endl;
    std::cout << "do not interfere with one another." << std::endl;
    std::cout << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "Calibrate X+" <<std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "Tracker frame Detected" << std::endl;
    std::cout << "   " << source_frame_ << std::endl;
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
    std::cout << "Y+ = Forward:" << std::endl;
    print_vector(y_direction);
    std::cout << "Z+ = Up:" << std::endl;
    print_vector(z_direction);

    Eigen::Matrix3d R_steamvr_from_teleop;
    R_steamvr_from_teleop.col(0) = x_direction;
    R_steamvr_from_teleop.col(1) = y_direction;
    R_steamvr_from_teleop.col(2) = z_direction;

    Eigen::Matrix3d R_teleop_from_steamvr = R_steamvr_from_teleop.transpose();

    Eigen::Quaterniond quaternion(R_teleop_from_steamvr);
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
    static_transform.child_frame_id = source_frame_;

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
    std::cout << "Tracker source frame:" << std::endl;
    std::cout << "  " << source_frame_ << std::endl;

    std::cout << std::endl;

    std::cout << "Published:" << std::endl;
    std::cout << "  " << target_frame_ << " -> " << source_frame_ << std::endl;
    std::cout << std::endl;

    std::cout << "Coordinate Conversion:" << std::endl;
    std::cout << "  X+ = Right" << std::endl;
    std::cout << "  Y+ = Forward" << std::endl;
    std::cout << "  Z+ = Up" << std::endl;
    std::cout << "=============================================" << std::endl;\

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

  void save_calibration(
    const Eigen::Vector3d & x_direction,
    const Eigen::Vector3d & y_direction,
    const Eigen::Vector3d & z_direction,
    const Eigen::Quaterniond & quaternion)
  {
    const char * home_dir = std::getenv("HOME");

    if (home_dir == nullptr)
    {
      RCLCPP_ERROR(get_logger(), "Could not get home directory.");
      return;
    }

    const std::filesystem::path config_dir = std::filesystem::path(home_dir) / ".config" / "annin_ar4_teleoperator";
    const std::filesystem::path calibration_file = config_dir / "calibration.yaml";

    try
    {
      std::filesystem::create_directories(config_dir);
    }
    catch(const std::filesystem::filesystem_error & ex)
    {
      RCLCPP_ERROR(get_logger(), "Could not create calibration directory: %s", ex.what());

      return;
    }

    YAML::Emitter out;

    out <<YAML::BeginMap;

    out << YAML::Key << "calibration";
    out << YAML::BeginMap;

    //Frame names
    out << YAML::Key << "target_frame"
        << YAML::Value << target_frame_;
    out << YAML::Key << "source_frame"
        << YAML::Value << source_frame_;

    
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

    //Rotation
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
        RCLCPP_ERROR(get_logger(), "Could not open calibration file for writing: %s", calibration_file.c_str());
        return;
      }

      file << out.c_str();
      file.close();
    }
    catch(const std::exception & ex)
    {
      RCLCPP_ERROR(get_logger(), "Could not save calibration: %s", ex.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "Calibration saved to: %s", calibration_file.c_str());
    
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<MovementCalibration>(node_options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}