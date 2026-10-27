/**
* @file tracker_udp_bridge.cpp
* @brief A C++ script for receiving UDP messages and publishing them to ROS2
*
* It receives tracker data over UDP and publishes the latest tracker
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
* @date October 07 - 2026
*/

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <atomic>
#include <stdexcept>
#include <sys/time.h>
#include <functional>
#include <cstdint>

class TrackerUdpBridge: public rclcpp::Node
{
public:

  TrackerUdpBridge()
    : Node("tracker_udp_bridge"),
      running_(true),
      have_data_(false),
      received_packets_(0),
      published_packets_(0),
      timer_calls_(0),
      sequence_jumps_(0),
      last_sequence_(0),
      max_publish_interval_(0.0)
  {
    udp_ip_ = declare_parameter<std::string>("udp_ip", "0.0.0.0");
    udp_port_ = declare_parameter<int>("udp_port", 5005);

    update_rate_ = 100.0;
    max_data_age_ = 0.1;


    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/tracker/pose", 1);

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/tracker/velocity", 1);


    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_fd_ < 0)
    {
      throw std::runtime_error("Failed to create UDP socket");
    }

    int reuse = 1;

    if (setsockopt(
      socket_fd_,
      SOL_SOCKET,
      SO_REUSEADDR,
      &reuse,
      sizeof(reuse)) < 0)
    {
      close(socket_fd_);

      throw std::runtime_error("Failed to set SO_REUSEADDR");
    }


    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; //100 ms

    if (setsockopt(
      socket_fd_,
      SOL_SOCKET,
      SO_RCVTIMEO,
      &timeout,
      sizeof(timeout)) < 0)
    {
      close(socket_fd_);

      throw std::runtime_error("Failed to set UDP receive timeout");
    }


    sockaddr_in address{};

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(udp_port_);

    if (bind(
      socket_fd_,
      reinterpret_cast<sockaddr*>(&address),
      sizeof(address)) < 0)
    {
      close (socket_fd_);

      throw std::runtime_error("Failed to bind UDP socket to port");
    }


    auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / update_rate_));

    publish_timer_ = this->create_wall_timer(timer_period,
      std::bind(&TrackerUdpBridge::publish_latest_data, this));

    receiver_thread_ = std::thread(&TrackerUdpBridge::receive_udp_data, this);

    
    last_publish_time_ = std::chrono::steady_clock::now();

    last_diagnostic_time_ = std::chrono::steady_clock::now();


    RCLCPP_INFO(get_logger(),
      "Listening for UDP packets on %s:%d",
      udp_ip_.c_str(),
      udp_port_);

    RCLCPP_INFO(get_logger(),
      "ROS2 publishing rate: %.2f Hz",
      update_rate_);
  }

  ~TrackerUdpBridge()
  {
    shutdown();
  }

private:
  
  struct TrackerData
  {
    uint64_t sequence = 0;
    double timestamp = 0.0;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;

    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;

    std::chrono::steady_clock::time_point receive_time;
  };

  std::string udp_ip_;
  int udp_port_;

  int socket_fd_ = -1;

  double update_rate_;
  double max_data_age_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;

  std::thread receiver_thread_;
  std::atomic<bool> running_;

  TrackerData latest_data_;
  std::mutex data_mutex_;
  bool have_data_;

  std::atomic<uint64_t> received_packets_;
  std::atomic<uint64_t> published_packets_;
  std::atomic<uint64_t> timer_calls_;
  std::atomic<uint64_t> sequence_jumps_;
  
  uint64_t last_sequence_;
  uint64_t last_sequence_published_;

  std::chrono::steady_clock::time_point last_publish_time_;
  std::chrono::steady_clock::time_point last_diagnostic_time_;
  double max_publish_interval_;
  double min_diagnostic_interval_ = 1.0; // seconds


  void receive_udp_data()
  {
    char buffer[4096];

    while (running_)
    {
      sockaddr_in sender_address{};
      socklen_t sender_length = sizeof(sender_address);

      ssize_t length = recvfrom(
        socket_fd_,
        buffer,
        sizeof(buffer) -1,
        0,
        reinterpret_cast<sockaddr*>(&sender_address),
        &sender_length);


      if (length < 0)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          continue;
        }

        if(!running_)
        {
          break;
        }

        RCLCPP_WARN(get_logger(),
          "UDP recvfrom() error: %s",
          std::strerror(errno));

        continue;
      }

      if (length == 0)
      {
        continue;
      }

      buffer[length] = '\0';


      TrackerData packet;

      if (!parse_packet(std::string(buffer), packet))
      {
        continue;
      }

      packet.receive_time = std::chrono::steady_clock::now();

      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (have_data_)
        {
          uint64_t expected = last_sequence_ + 1;

          if (packet.sequence <= last_sequence_)
          {
            continue;
          }

          if (packet.sequence != expected)
          {
            sequence_jumps_++;
          }
        }

        last_sequence_ = packet.sequence;


      

        latest_data_ = packet;
        have_data_ = true;
      }

      received_packets_++;
    }
  }

  
  bool parse_packet(
    const std::string& message,
    TrackerData& packet)
  {
    std::stringstream stream(message);

    std::string field;

    std::vector<std::string> values;

    while (std::getline(
      stream,
      field,
      ','))
    {
      values.push_back(field);
    }

    if (values.size() != 15)
    {
      return false;
    }


    try
    {
      packet.sequence = std::stoull(values[0]);
      packet.timestamp = std::stod(values[1]);

      packet.x = std::stod(values[2]);
      packet.y = std::stod(values[3]);
      packet.z = std::stod(values[4]);

      packet.qx = std::stod(values[5]);
      packet.qy = std::stod(values[6]);
      packet.qz = std::stod(values[7]);
      packet.qw = std::stod(values[8]);

      packet.vx = std::stod(values[9]);
      packet.vy = std::stod(values[10]);
      packet.vz = std::stod(values[11]);

      packet.wx = std::stod(values[12]);
      packet.wy = std::stod(values[13]);
      packet.wz = std::stod(values[14]);

    }
    catch(const std::exception&)
    {
      return false;
    }

    return true;
    
  }


  void publish_latest_data()
  {
    const auto current_time = std::chrono::steady_clock::now();

    const double dt = std::chrono::duration<double>(current_time - last_publish_time_).count();

    if (dt > max_publish_interval_)
    {
      max_publish_interval_ = dt;
    }

    last_publish_time_ = current_time;

    timer_calls_++;


    TrackerData data;

    bool have_data;
    
    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      have_data = have_data_;

      if (have_data)
      {
        data = latest_data_;
      }
    }

    if (!have_data)
    {
      print_diagnostics();
      return;
    }


    const double data_age = std::chrono::duration<double>(current_time - data.receive_time).count();

    if (data_age > max_data_age_)
    {
      publish_zero_velocity();

      print_diagnostics();

      return;
    }

    if (last_sequence_published_ == data.sequence)
    {
      return;
    }
    
    auto stamp = this->get_clock()->now();

    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "steamvr_world";

    pose_msg.pose.position.x = data.x;
    pose_msg.pose.position.y = data.y;
    pose_msg.pose.position.z = data.z;

    pose_msg.pose.orientation.x = data.qx;
    pose_msg.pose.orientation.y = data.qy;
    pose_msg.pose.orientation.z = data.qz;
    pose_msg.pose.orientation.w = data.qw;

    pose_publisher_->publish(pose_msg);


    geometry_msgs::msg::TwistStamped velocity_msg;

    velocity_msg.header.stamp = stamp;
    velocity_msg.header.frame_id = "steamvr_world";

    velocity_msg.twist.linear.x = data.vx;
    velocity_msg.twist.linear.y = data.vy;
    velocity_msg.twist.linear.z = data.vz;

    velocity_msg.twist.angular.x = data.wx;
    velocity_msg.twist.angular.y = data.wy;
    velocity_msg.twist.angular.z = data.wz;

    velocity_publisher_->publish(velocity_msg);

    published_packets_++;

    last_sequence_published_ = data.sequence;

    // RCLCPP_INFO(get_logger(),
    //   "Publishing seq=%lu | vx = %.4f | vy = %.4f | vz = %.4f ",
    //   data.sequence, data.vx, data.vy, data.vz);

    print_diagnostics();
  }

  void publish_zero_velocity()
  {
    geometry_msgs::msg::TwistStamped zero_msg;

    zero_msg.header.stamp = this->get_clock()->now();
    zero_msg.header.frame_id = "steamvr_world";

    zero_msg.twist.linear.x = 0.0;
    zero_msg.twist.linear.y = 0.0;
    zero_msg.twist.linear.z = 0.0;

    zero_msg.twist.angular.x = 0.0;
    zero_msg.twist.angular.y = 0.0;
    zero_msg.twist.angular.z = 0.0;

    velocity_publisher_->publish(zero_msg);
  }


  void print_diagnostics()
  {
    const auto now = std::chrono::steady_clock::now();

    const double elapsed = std::chrono::duration<double>(now - last_diagnostic_time_).count();

    if (elapsed < min_diagnostic_interval_)
    {
      return;
    }

    last_diagnostic_time_ = now;

    RCLCPP_INFO(get_logger(),
      "RX: %6lu | "
      "PUB: %6lu | "
      "TIMER: %6lu | "
      "SEQ_JUMPS: %4lu | "
      "LATEST_SEQ: %lu | "
      "MAX_PUB_DT: %.4f ms",
      
      received_packets_.load(),
      published_packets_.load(),
      timer_calls_.load(),
      sequence_jumps_.load(),
      last_sequence_,
      max_publish_interval_ * 1000.0);
  }

  void shutdown()
  {
    if (!running_)
    {
      return;
    }

    running_ = false;

    if (socket_fd_ >= 0)
    {
      ::shutdown(socket_fd_, SHUT_RDWR);

      close(socket_fd_);

      socket_fd_ = -1;
    }

    if (receiver_thread_.joinable())
    {
      receiver_thread_.join();
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TrackerUdpBridge>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}