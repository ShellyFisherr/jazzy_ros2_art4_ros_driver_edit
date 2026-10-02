/**
 * @file frame_translator.cpp
 * @brief A script for translating the HTCVive tracker coordinate frame to the AR4 coordinate frame system
 * 
 * -------------------------
 * Publishing Topics
 * 
 * -------------------------
 * Subscribed Topics
 * 
 * @author Daan Krijnen
 * @date September 23, 2026
 */

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"


class FrameTranslator : public rclcpp::Node
{
public:
    FrameTranslator(const rclcpp::NodeOptions & node_options) 
        : Node("frame_translator", node_options)
    {
        target_frame_ = this->declare_parameter<std::string>(
            "target_frame", "teleop_world");

        velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/tracker/velocity", 10, std::bind(&FrameTranslator::velocity_callback, this, std::placeholders::_1));
        
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/velocity_world", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Target frame: %s",
            target_frame_.c_str());
    }


private:

    std::string target_frame_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    
    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        const std::string source_frame = msg->header.frame_id;

        if (source_frame.empty())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Received velocity with empty frame_id.");

            return;
        }

        geometry_msgs::msg::TransformStamped transform;

        try
        {
            transform = tf_buffer_->lookupTransform(target_frame_,
                                                    source_frame,
                                                    msg->header.stamp,
                                                    rclcpp::Duration::from_seconds(0.05));
        }
        catch(const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Could not transform velocity frame from '%s' to '%s' : %s",
                source_frame.c_str(), target_frame_.c_str(), ex.what());

            return;
        }

        geometry_msgs::msg::TwistStamped output;

        output.header.stamp = msg->header.stamp;
        output.header.frame_id = target_frame_;


        tf2::doTransform(msg->twist.linear,
                        output.twist.linear,
                        transform);

        tf2::doTransform(msg->twist.angular,
                        output.twist.angular,
                        transform);

        velocity_publisher_->publish(output);
    }
};

int main (int argc, char * argv[])
{
    std::cerr << "========== Frame Translator Started ==========" << std::endl;

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<FrameTranslator>(node_options);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
