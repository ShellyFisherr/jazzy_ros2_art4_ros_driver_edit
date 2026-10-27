/**
* @file openvr_tracker.cpp
* @brief A C++ script for receiving OpenVR SDK messages and publishing them to ROS2
*
* It receives tracker data from the OpenVR SDK and converts it to ROS2 topics
* position and velocity to the respective ROS2 topics.
*
* --------------------------------------------
* Subscribed Topics:
*
* None
*
* --------------------------------------------
* Published Topics:
*
* /tracker/pose             geometry_msgs::msg::PoseStamped         (Position of steamVR tracker in tracker frame)
* /tracker/velocity         geometry_msgs::msg::TwistStamped        (Velocity of steamVR tracker in tracker frame)
*
* --------------------------------------------
* @author Daan Krijnen
* @date October 26 - 2026
*/


#include <openvr.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

struct Quaternion
{
  double w;
  double x;
  double y;
  double z;
};

Quaternion rotation_matrix_to_quaternion(const vr::HmdMatrix34_t & matrix)
{
  const double r00 = matrix.m[0][0];
  const double r01 = matrix.m[0][1];
  const double r02 = matrix.m[0][2];

  const double r10 = matrix.m[1][0];
  const double r11 = matrix.m[1][1];
  const double r12 = matrix.m[1][2];

  const double r20 = matrix.m[2][0];
  const double r21 = matrix.m[2][1];
  const double r22 = matrix.m[2][2];

  Quaternion q{};
  const double trace = r00 + r11 + r22;

  if (trace > 0.0)
  {
    const double s = 0.5 / std::sqrt(trace + 1.0);

    q.w = 0.25 / s;
    q.x = (r21 - r12) * s;
    q.y = (r02 - r20) * s;
    q.z = (r10 - r01) * s;
  }
  else if (r00 > r11 && r00 > r22)
  {
    const double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);

    q.w = (r21 - r12) / s;
    q.x = 0.25 * s;
    q.y = (r01 + r10) / s;
    q.z = (r02 + r20) / s;
  }
  else if (r11 > r22)
  {
    const double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);

    q.w = (r02 - r20) / s;
    q.x = (r01 + r10) / s;
    q.y = 0.25 * s;
    q.z = (r12 + r21) / s;
  }
  else
  {
    const double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);

    q.w = (r10 - r01) / s;
    q.x = (r02 + r20) / s;
    q.y = (r12 + r21) / s;
    q.z = 0.25 * s;
  }

  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

  if (!std::isfinite(norm))
  {
    throw std::runtime_error("OpenVR porduced invalid orientation quaternion");
  }

  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;

  return q;
}

class OpenVrSession
{
public:
  OpenVrSession()
  {
    vr::EVRInitError error = vr::VRInitError_None;
    system_ = vr::VR_Init(&error, vr::VRApplication_Background);

    if (system_ == nullptr || error != vr::VRInitError_None)
    {
      throw std::runtime_error(
        std::string("OpenVR initialization failed: ") +
        vr::VR_GetVRInitErrorAsEnglishDescription(error));
    }
  }
  
  ~OpenVrSession()
  {
    if (system_ != nullptr)
    {
      vr::VR_Shutdown();
    }
  }

  OpenVrSession(const OpenVrSession &) = delete;
  OpenVrSession & operator=(const OpenVrSession &) = delete;

  vr::IVRSystem * system() const
  {
    return system_;
  }

private:
  vr::IVRSystem * system_{nullptr};
};



class OpenVrTracker : public rclcpp::Node
{
public:
  OpenVrTracker() : Node("openvr_tracker")
  {
    tracker_index_ = find_first_tracker();

    if (tracker_index_ == vr::k_unTrackedDeviceIndexInvalid)
    {
      throw std::runtime_error("No connected OpenVR Generic Tracker was found");
    }

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/tracker/pose", rclcpp::QoS(10));

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/tracker/velocity", rclcpp::QoS(10));

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&OpenVrTracker::publish_tracker_data, this));

    RCLCPP_INFO(get_logger(),
      "OpenVR tracker found at device index %u; publishing at 100 Hz",
      tracker_index_);
  }

private:
  OpenVrSession openvr_;
  vr::TrackedDeviceIndex_t tracker_index_{vr::k_unTrackedDeviceIndexInvalid};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  vr::TrackedDeviceIndex_t find_first_tracker() const
  {
    vr::IVRSystem * system = openvr_.system();

    for (vr::TrackedDeviceIndex_t index = 0; index < vr::k_unMaxTrackedDeviceCount; ++index)
    {
      if (system->IsTrackedDeviceConnected(index) &&
          system->GetTrackedDeviceClass(index) == vr::TrackedDeviceClass_GenericTracker)
      {
        return index;
      }
    }

    return vr::k_unTrackedDeviceIndexInvalid;
  }

  void publish_zero_velocity()
  {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = now();
    message.header.frame_id = "steamvr_world";

    velocity_publisher_->publish(message);
  }

  void publish_tracker_data()
  {
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};

    openvr_.system()->GetDeviceToAbsoluteTrackingPose(
      vr::TrackingUniverseStanding,
      0.0f,
      poses,
      vr::k_unMaxTrackedDeviceCount);

    const vr::TrackedDevicePose_t & pose = poses[tracker_index_];

    if (!pose.bPoseIsValid)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "OpenVR tracker pose in invalid; publishing zero velocity");

      publish_zero_velocity();
      return;
    }

    const vr::HmdMatrix34_t & transform = pose.mDeviceToAbsoluteTracking;

    const double x = transform.m[0][3];
    const double y = transform.m[1][3];
    const double z = transform.m[2][3];

    const double vx = pose.vVelocity.v[0];
    const double vy = pose.vVelocity.v[1];
    const double vz = pose.vVelocity.v[2];

    const double wx = pose.vAngularVelocity.v[0];
    const double wy = pose.vAngularVelocity.v[1];
    const double wz = pose.vAngularVelocity.v[2];

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) ||
        !std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "OpenVR returned non-finite tracker data; publishing zero velocity");

      publish_zero_velocity();
      return;
    }

    Quaternion orientation;

    try
    {
      orientation = rotation_matrix_to_quaternion(transform);
    }
    catch(const std::runtime_error & error)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "%s; publishing zero velocity",
        error.what());

      publish_zero_velocity();
      return;
    }

    const rclcpp::Time stamp = now();

    geometry_msgs::msg::PoseStamped pose_message;
    pose_message.header.stamp = stamp;
    pose_message.header.frame_id = "steamvr_world";

    pose_message.pose.position.x = x;
    pose_message.pose.position.y = y;
    pose_message.pose.position.z = z;

    pose_message.pose.orientation.x = orientation.x;
    pose_message.pose.orientation.y = orientation.y;
    pose_message.pose.orientation.z = orientation.z;
    pose_message.pose.orientation.w = orientation.w;

    geometry_msgs::msg::TwistStamped velocity_message;
    velocity_message.header.stamp = stamp;
    velocity_message.header.frame_id = "steamvr_world";

    velocity_message.twist.linear.x = vx;
    velocity_message.twist.linear.y = vy;
    velocity_message.twist.linear.z = vz;

    velocity_message.twist.angular.x = wx;
    velocity_message.twist.angular.y = wy;
    velocity_message.twist.angular.z = wz;

    pose_publisher_->publish(pose_message);
    velocity_publisher_->publish(velocity_message);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<OpenVrTracker>();
    rclcpp::spin(node);
  }
  catch(const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("openvr_tracker"),
    "%s", error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}