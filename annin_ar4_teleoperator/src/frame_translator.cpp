/**
 * @file frame_translator.cpp
 * @brief A script for translating the HTCVive tracker coordinate frame from SteamVR to the AR4 coordinate frame system
 * 
 * Calibration loads the file from
 *  ~/.config/annin_ar4_teleoperator/calibration.yaml
 * 
 * The calibration quaternion stored in the .YAML file represents the rotation from the tracker frame to the AR4 coordinate frame.
 * 
 * Since this node transforms velocities, only the rotation is needed, and the translation is not used.
 * 
 * -------------------------
 * Publishing Topics
 *  /velocity_world          geometry_msgs::msg::TwistStamped         (transformed velocity in editted AR4 frame)
 *  /position_world          geometry_msgs::msg::PoseStamped          (transformed position in editted AR4 frame)
 * 
 * -------------------------
 * Subscribed Topics
 * /tracker/pose             geometry_msgs::msg::PoseStamped         (Position of steamVR tracker in tracker frame)
 * /tracker/velocity         geometry_msgs::msg::TwistStamped        (Velocity of steamVR tracker in tracker frame)
 * 
 * @author Daan Krijnen
 * @date October 15, 2026
 */


#include <memory>
#include <string>
#include <cstdlib>
#include <filesystem>
// #include <yaml-cpp/yaml.h>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

class FrameTranslator : public rclcpp::Node
{
public:
  FrameTranslator (const rclcpp::NodeOptions & node_options)
    : Node("frame_translator", node_options)
  {

    if (!load_parameters())
    {
      throw std::runtime_error("could not load calibration file");
    }

    velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/tracker/velocity", 10, std::bind(&FrameTranslator::velocity_callback, this, std::placeholders::_1));

    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tracker/pose", 10, std::bind(&FrameTranslator::pose_callback, this, std::placeholders::_1));
    
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/velocity_world", 10);
    
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/position_world", 10);

    RCLCPP_INFO(get_logger(),
      "=======================================");

    RCLCPP_INFO(get_logger(),
      "      Frame Translator Started");

    RCLCPP_INFO(get_logger(),
      "=======================================");

    RCLCPP_INFO(get_logger(),
      "Target frame: %s", target_frame_.c_str());

    RCLCPP_INFO(get_logger(),
      "Calibration rotation loaded successfully.");
    RCLCPP_INFO(get_logger(),
      "Applying calibration as quaternion.");

    RCLCPP_INFO(get_logger(),
      "=======================================");
  }

private:

  std::string target_frame_;
  std::string source_frame_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  tf2::Quaternion quaternion_;

  tf2::Vector3 translation_;

  double alpha_ = 0.3509;

  double vx_prev_ = 0.0;
  double vy_prev_ = 0.0;
  double vz_prev_ = 0.0;

  double wx_prev_ = 0.0;
  double wy_prev_ = 0.0;
  double wz_prev_ = 0.0;

  bool load_parameters()
  {
    RCLCPP_INFO(get_logger(),
      "Loading calibration.yaml file...");

    target_frame_ = declare_parameter<std::string>("target_frame", "teleop_world");

    if (target_frame_.empty())
    {
      RCLCPP_FATAL(get_logger(),
        "target_frame cannot be empty");
      return false;
    }

    source_frame_ = declare_parameter<std::string>("source_frame", "steamvr_world");

    if (source_frame_.empty())
    {
      RCLCPP_FATAL(get_logger(),
        "source_frame cannot be empty");
      return false;
    }

    const std::vector<double> rotation_xyzw =
      declare_parameter<std::vector<double>>("rotation_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});

    if (rotation_xyzw.size() != 4)
    {
      RCLCPP_FATAL(get_logger(),
        "rotation_xyzw must contain [x, y, z, w]");

      return false;
    }

    quaternion_.setValue(
      rotation_xyzw[0],
      rotation_xyzw[1],
      rotation_xyzw[2],
      rotation_xyzw[3]);

    if (quaternion_.length2() < 1e-12)
    {
      RCLCPP_FATAL(get_logger(),
      "rotation_xyzw cannot be a zero quaternion.");
      return false;
    }

    quaternion_.normalize();

    const std::vector<double> translation_xyz =
      declare_parameter<std::vector<double>>("translation_xyz", std::vector<double>{0.0, 0.0, 0.0});

    if (translation_xyz.size() != 3)
    {
      RCLCPP_FATAL(get_logger(),
        "translation_xyz must contain [x, y, z]");

      return false;
    }

    for (const double component : translation_xyz)
    {
      if (!std::isfinite(component))
      {
        RCLCPP_FATAL(get_logger(),
          "translation_xyz must be finite");

        return false;
      }
    }

    translation_.setValue(
      translation_xyz[0],
      translation_xyz[1],
      translation_xyz[2]);

    RCLCPP_INFO(get_logger(),"========================================");
    RCLCPP_INFO(get_logger(),"Loaded calibration quaternion:");
    RCLCPP_INFO(get_logger()," x: %.12f", quaternion_.x());
    RCLCPP_INFO(get_logger()," y: %.12f", quaternion_.y());
    RCLCPP_INFO(get_logger()," z: %.12f", quaternion_.z());
    RCLCPP_INFO(get_logger()," w: %.12f", quaternion_.w());
    RCLCPP_INFO(get_logger(),"========================================");
    RCLCPP_INFO(get_logger(),"Loaded translation vector:");
    RCLCPP_INFO(get_logger()," x: %.12f", translation_.x());
    RCLCPP_INFO(get_logger()," y: %.12f", translation_.y());
    RCLCPP_INFO(get_logger()," z: %.12f", translation_.z());
    RCLCPP_INFO(get_logger(),"========================================");

    return true;
  }


  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received pose with empty frame_id");

      return;
    }

    const tf2::Vector3 source_position(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

    const tf2::Vector3 target_position = tf2::quatRotate(quaternion_, source_position) + translation_;

    geometry_msgs::msg::PoseStamped output;
    output.header = msg->header;
    output.header.frame_id = target_frame_;

    output.pose.position.x = target_position.x();
    output.pose.position.y = target_position.y();
    output.pose.position.z = target_position.z();

    tf2::Quaternion source_orientation(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);

    const tf2::Quaternion target_orientation = quaternion_ * source_orientation;

    output.pose.orientation.x = target_orientation.x();
    output.pose.orientation.y = target_orientation.y();
    output.pose.orientation.z = target_orientation.z();
    output.pose.orientation.w = target_orientation.w();

    pose_publisher_->publish(output);
  }


  void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (msg->header.frame_id.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received velocity with empty frame_id.");

      return;
    }

    const double vx_raw = msg->twist.linear.x;
    const double vy_raw = msg->twist.linear.y;
    const double vz_raw = msg->twist.linear.z;

    const double wx_raw = msg->twist.angular.x;
    const double wy_raw = msg->twist.angular.y;
    const double wz_raw = msg->twist.angular.z;

    const double vx_filter = alpha_ * vx_raw + (1 - alpha_) * vx_prev_;
    const double vy_filter = alpha_ * vy_raw + (1 - alpha_) * vy_prev_;
    const double vz_filter = alpha_ * vz_raw + (1 - alpha_) * vz_prev_;

    const double wx_filter = alpha_ * wx_raw + (1 - alpha_) * wx_prev_;
    const double wy_filter = alpha_ * wy_raw + (1 - alpha_) * wy_prev_;
    const double wz_filter = alpha_ * wz_raw + (1 - alpha_) * wz_prev_;

    vx_prev_ = vx_filter;
    vy_prev_ = vy_filter;
    vz_prev_ = vz_filter;

    wx_prev_ = wx_filter;
    wy_prev_ = wy_filter;
    wz_prev_ = wz_filter;

    const tf2::Vector3 linear_velocity_tracker(vx_filter, vy_filter, vz_filter);

    const tf2::Vector3 angular_velocity_tracker(wx_filter, wy_filter, wz_filter);

    
    const tf2::Vector3 linear_velocity_world = tf2::quatRotate(quaternion_, linear_velocity_tracker);
    const tf2::Vector3 angular_velocity_world = tf2::quatRotate(quaternion_, angular_velocity_tracker);

    geometry_msgs::msg::TwistStamped output;

    output.header.stamp = msg->header.stamp;

    output.header.frame_id = target_frame_;

    output.twist.linear.x = linear_velocity_world.x();
    output.twist.linear.y = linear_velocity_world.y();
    output.twist.linear.z = linear_velocity_world.z();

    output.twist.angular.x = angular_velocity_world.x();
    output.twist.angular.y = angular_velocity_world.y();
    output.twist.angular.z = angular_velocity_world.z();

    velocity_publisher_->publish(output);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("use_sim_time", false);

  auto node = std::make_shared<FrameTranslator>(node_options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}