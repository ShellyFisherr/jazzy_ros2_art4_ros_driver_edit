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
 *  /velocity_world         geometry_msgs::msg::TwistStamped     (transformed velocity in AR4 coordinate frame)
 * 
 * -------------------------
 * Subscribed Topics
 *  /tracker/velocity         geometry_msgs::msg::TwistStamped     (velocity from SteamVR tracker)
 * 
 * @author Daan Krijnen
 * @date October 02, 2026
 */


#include <memory>
#include <string>
#include <cstdlib>
#include <filesystem>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

class FrameTranslator : public rclcpp::Node
{
public:
  FrameTranslator (const rclcpp::NodeOptions & node_options)
    : Node("frame_translator", node_options)
  {
    target_frame_ = this->declare_parameter<std::string>(
      "target_frame", "teleop_world");

    if (!load_calibration())
    {
      RCLCPP_FATAL(get_logger(), "Failed to load calibration. Exiting.");

      throw std::runtime_error("Could not load calibration file.");
    }

    velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/tracker/velocity", 10, std::bind(&FrameTranslator::velocity_callback, this, std::placeholders::_1));
    
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/velocity_world", 10);

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

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;

  tf2::Quaternion quaternion;

  bool load_calibration()
  {
    try
    {
      const char * home_dir = std::getenv("HOME");

      if (home_dir == nullptr)
      {
        RCLCPP_ERROR(get_logger(), "Could not get home directory.");
        return false;
      }

      const std::filesystem::path calibration_path = std::filesystem::path(home_dir)
        / ".config" / "annin_ar4_teleoperator" / "calibration.yaml";

      RCLCPP_INFO(get_logger(), "Loading calibration from: %s", calibration_path.c_str());

      if (!std::filesystem::exists(calibration_path))
      {
        RCLCPP_ERROR(get_logger(), "Calibration file does not exist: %s", calibration_path.c_str());
        return false;
      }

      YAML::Node config = YAML::LoadFile(calibration_path.string());

      if (!config["calibration"])
      {
        RCLCPP_ERROR(get_logger(), "Calibration file does not have 'calibration' section.");
        return false;
      }

      YAML::Node calibration = config["calibration"];


      if (calibration["target_frame"])
      {
        const std::string yaml_target = calibration["target_frame"].as<std::string>();

        RCLCPP_INFO(get_logger(), "Calibration target frame: %s", yaml_target.c_str());
      }

      if (calibration["source_frame"])
      {
        const std::string yaml_source = calibration["source_frame"].as<std::string>();

        RCLCPP_INFO(get_logger(), "Calibration source frame: %s", yaml_source.c_str());
      }


      if (!calibration["rotation"])
      {
        RCLCPP_ERROR(get_logger(), "Calibration file does not have 'rotation' section.");
        return false;
      }

      YAML::Node rotation = calibration["rotation"];

      if (!rotation["x"] || !rotation["y"] || !rotation["z"] || !rotation["w"])
      {
        RCLCPP_ERROR(get_logger(), "Calibration rotation is missing components.");
        return false;
      }

      const double qx = rotation["x"].as<double>();
      const double qy = rotation["y"].as<double>();
      const double qz = rotation["z"].as<double>();
      const double qw = rotation["w"].as<double>();

      quaternion.setValue(qx, qy, qz, qw);
      quaternion.normalize();

      RCLCPP_INFO(get_logger()," ========================================");
      RCLCPP_INFO(get_logger()," Loaded calibration quaternion:");
      RCLCPP_INFO(get_logger(),"  x: %.12f", quaternion.x());
      RCLCPP_INFO(get_logger(),"  y: %.12f", quaternion.y());
      RCLCPP_INFO(get_logger(),"  z: %.12f", quaternion.z());
      RCLCPP_INFO(get_logger(),"  w: %.12f", quaternion.w());
      RCLCPP_INFO(get_logger()," ========================================");

      return true;
    }
    catch(const YAML::Exception & ex)
    {
      RCLCPP_ERROR(get_logger(),"YAML error while loading calibration: %s", ex.what());
      return false;
    }
    catch(const std::exception & ex)
    {
      RCLCPP_ERROR(get_logger(),"Error while loading calibration: %s", ex.what());
      return false;
    }
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

    const tf2::Vector3 linear_velocity_tracker(
      msg->twist.linear.x,
      msg->twist.linear.y,
      msg->twist.linear.z);

    const tf2::Vector3 angular_velocity_tracker(
      msg->twist.angular.x,
      msg->twist.angular.y,
      msg->twist.angular.z);

    
    const tf2::Vector3 linear_velocity_world = tf2::quatRotate(quaternion, linear_velocity_tracker);
    const tf2::Vector3 angular_velocity_world = tf2::quatRotate(quaternion, angular_velocity_tracker);

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
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<FrameTranslator>(node_options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}