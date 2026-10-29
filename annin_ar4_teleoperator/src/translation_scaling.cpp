/**
 * @file translation_scaling.cpp
 * @brief A system for calibrating the movement scale between robot and human operator. It loads, calculates and saves required data automatically. 
 * 
 * 
 * -------------------------
 * Publishing Topics
 * 
 * None
 * 
 * -------------------------
 * Subscribed Topics
 * 
 * pose_subscription_       /tracker/pose           geometry_msgs::msg::PoseStamped                 (Tracker position in 3d space)
 * 
 * @author Daan Krijnen
 * @date October 29, 2026
 */


#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

class TranslationScaling : public rclcpp::Node
{
public:

    explicit TranslationScaling(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("translation_scaling", node_options)
    {
        load_parameters();

        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracker/pose", 10, std::bind(&TranslationScaling::pose_callback, this, std::placeholders::_1));

        calibration_thread_ = std::thread(&TranslationScaling::run_calibration, this);

        RCLCPP_INFO(get_logger(),
            "==============================================");
        RCLCPP_INFO(get_logger(),
            "Translation scaling calibration started");
        RCLCPP_INFO(get_logger(),
            "==============================================");
        RCLCPP_INFO(get_logger(),
            "Tracker frame: %s",
            tracker_frame_.c_str());
        RCLCPP_INFO(get_logger(),
            "Target frame: %s",
            target_frame_.c_str());
        
    }

    ~TranslationScaling()
    {
        if (calibration_thread_.joinable())
        {
            calibration_thread_.join();
        }
    }

private:

    class RestartCalibrationError : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    std::string target_frame_;
    std::string tracker_frame_;
    std::filesystem::path result_file_;

    Eigen::Vector3d robot_near_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d robot_far_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d tracker_to_target_translation_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond tracker_to_target_rotation_ = Eigen::Quaterniond::Identity();

    std::size_t capture_sample_count_;
    double minimum_tracker_movement_;
    double minimum_robot_movement_;
    double maximum_capture_spread_;

    std::mutex tracker_mutex_;
    std::condition_variable tracker_condition_;
    std::vector<Eigen::Vector3d> capture_samples_;
    bool received_pose_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

    std::thread calibration_thread_;

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
        target_frame_ = declare_parameter<std::string>("target_frame", "teleop_world");

        tracker_frame_ = declare_parameter<std::string>("tracker_frame", "steamvr_world");

        const std::vector<double> robot_near_values =
            declare_parameter<std::vector<double>>("robot_near", std::vector<double>{0.0, 0.0, 0.0});

        const std::vector<double> robot_far_values =
            declare_parameter<std::vector<double>>("robot_far", std::vector<double>{0.0, 0.0, 0.0});

        const std::vector<double> rotation_xyzw =
            declare_parameter<std::vector<double>>("tracker_to_target_rotation_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});

        const std::vector<double> translation_xyz =
            declare_parameter<std::vector<double>>("tracker_to_target_translation_xyz", std::vector<double>{0.0, 0.0, 0.0});
        
        const int sample_count =
            declare_parameter<int>("capture_sample_count", 30);

        minimum_tracker_movement_ = declare_parameter<double>("minimum_tracker_movement", 0.05);

        minimum_robot_movement_ = declare_parameter<double>("minimum_robot_movement", 0.01);

        maximum_capture_spread_ = declare_parameter<double>("maximum_capture_spread", 0.02);

        result_file_ = declare_parameter<std::string>("result_file", "translation_scaling_result.yaml");

        if (target_frame_.empty())
        {
            throw std::runtime_error("target_frame cannot be empty");
        }

        if (tracker_frame_.empty())
        {
            throw std::runtime_error("tracker_frame cannot be empty");
        }

        if (robot_near_values.size() != 3 || robot_far_values.size() != 3)
        {
            throw std::runtime_error("robot_near and robot_far must have exactly 3 values each.");
        }

        if (!std::isfinite(sample_count) || sample_count <= 0)
        {
            throw std::runtime_error("sample_count must be positive");
        }

        if (!std::isfinite(minimum_tracker_movement_) || minimum_tracker_movement_ <= 0.0)
        {
            throw std::runtime_error("minimum_tracker_movement must be finite and positive");
        }

        if (!std::isfinite(minimum_robot_movement_) || minimum_robot_movement_ <= 0.0)
        {
            throw std::runtime_error("minimum_robot_movement must be finite and positive");
        }

        if (!std::isfinite(maximum_capture_spread_) || maximum_capture_spread_ <= 0.0)
        {
            throw std::runtime_error("maximum_capture_spread must be finite and positive");
        }

        capture_sample_count_ = static_cast<std::size_t>(sample_count);

        robot_near_ = vector_from_parameter(robot_near_values, "robot_near");
        robot_far_ = vector_from_parameter(robot_far_values, "robot_far");
        tracker_to_target_translation_ = vector_from_parameter(translation_xyz, "tracker_to_target_translation_xyz");

        for (const double component : rotation_xyzw)
        {
            if (!std::isfinite(component))
            {
                throw std::runtime_error("Rotation quaternion must have finite components");
            }
        }

        tracker_to_target_rotation_ = Eigen::Quaterniond(
            rotation_xyzw[3],
            rotation_xyzw[0],
            rotation_xyzw[1],
            rotation_xyzw[2]);

        if (tracker_to_target_rotation_.norm() < 1e-12)
        {
            throw std::runtime_error("Tracker to target quaternion cannot be zero.");
        }

        tracker_to_target_rotation_.normalize();

        if (result_file_.empty())
        {
            throw std::runtime_error("result_file must not be empty.");
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (msg->header.frame_id != tracker_frame_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Ignoring pose in frame '%s'; expected '%s'.",
                msg->header.frame_id.c_str(),
                tracker_frame_.c_str());

            return;
        }

        const Eigen::Vector3d tracker_position(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z);

        if (!tracker_position.allFinite())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Ignoring tracker pose with non-finite positions.");
            
            return;
        }

        const Eigen::Vector3d target_position = tracker_to_target_rotation_ * tracker_position + tracker_to_target_translation_;

        {
            std::lock_guard<std::mutex> lock(tracker_mutex_);

            received_pose_ = true;

            if (capture_samples_.size() < capture_sample_count_)
            {
                capture_samples_.push_back(target_position);
            }
        }

        tracker_condition_.notify_all();
    }

    bool wait_for_enter(const std::string & prompt)
    {
        std::cout << "\n" << prompt << std::endl;
        std::cout << "Hold tracker still, then press ENTER." << std::endl;

        const int terminal_fd = ::open("/dev/tty", O_RDONLY);
        if (terminal_fd < 0)
        {
            throw std::runtime_error("Could not open controlling terminal /dev/tty.");
        }

        while (rclcpp::ok())
        {
            pollfd input_descriptor{};
            
            input_descriptor.fd = terminal_fd;
            input_descriptor.events = POLLIN;

            const int poll_result = ::poll(&input_descriptor, 1, 100);

            if (poll_result < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                ::close(terminal_fd);
                throw std::runtime_error("Error while waiting for terminal input.");
            }

            if (poll_result == 0)
            {
                continue;
            }

            if ((input_descriptor.revents & (POLLIN | POLLHUP)) != 0)
            {
                char character = '\0';
                const ssize_t bytes_read = ::read(terminal_fd, &character, sizeof(character));

                if (bytes_read <= 0)
                {
                    ::close(terminal_fd);
                    return false;
                }

                if (character == '\n')
                {
                    ::close(terminal_fd);
                    return true;
                }
            }
        }

        ::close(terminal_fd);
        return false;
    }

    void wait_for_tracker()
    {
        RCLCPP_INFO(get_logger(),
            "Waiting for tracker pose...");

        std::unique_lock<std::mutex> lock(tracker_mutex_);

        while (rclcpp::ok() && !received_pose_)
        {
            tracker_condition_.wait_for(lock, std::chrono::milliseconds(100));
        }
    }

    Eigen::Vector3d capture_position(const std::string & prompt)
    {
        if (!wait_for_enter(prompt))
        {
            throw std::runtime_error("Calibration Interupted");
        }

        RCLCPP_INFO(get_logger(),
            "Collecting %zu samples... please stand by.",
            capture_sample_count_);

        std::unique_lock<std::mutex> lock(tracker_mutex_);

        capture_samples_.clear();

        while (rclcpp::ok() && capture_samples_.size() < capture_sample_count_)
        {
            tracker_condition_.wait_for(lock, std::chrono::milliseconds(100));
        }

        if (!rclcpp::ok())
        {
            throw std::runtime_error("Calibration Interupted");
        }

        Eigen::Vector3d average = Eigen::Vector3d::Zero();

        for (const Eigen::Vector3d & sample : capture_samples_)
        {
            average += sample;
        }

        average /= static_cast<double>(capture_samples_.size());

        double maximum_deviation = 0.0;

        for (const Eigen::Vector3d & sample : capture_samples_)
        {
            maximum_deviation = std::max(maximum_deviation, (sample - average).norm());
        }

        if (maximum_deviation > maximum_capture_spread_)
        {
            throw RestartCalibrationError("Tracker moved during capture. Restart and hold it still.");
        }

        return average;
    }

    double calculate_scale(
        const Eigen::Vector3d & tracker_near_target,
        const Eigen::Vector3d & tracker_far_target) const
    {
        const Eigen::Vector3d tracker_delta = tracker_far_target - tracker_near_target;
        const Eigen::Vector3d robot_delta = robot_far_ - robot_near_;

        const double tracker_distance = tracker_delta.norm();
        const double robot_distance = robot_delta.norm();

        if (tracker_distance < minimum_tracker_movement_)
        {
            throw RestartCalibrationError("Tracker is below minimum Tracker Movement. Restarting...");
        }

        if (robot_distance < minimum_robot_movement_)
        {
            throw std::runtime_error("Robot is below minimum_robot_movement. Edit parameters.");
        }

        const double scale = robot_distance / tracker_distance;

        if (!std::isfinite(scale) || scale <= 0.0)
        {
            throw RestartCalibrationError("Calculated scale is invalid (negative). Restarting...");
        }

        return scale;
    }

    
    static void emit_vector3(
        YAML::Emitter & output,
        const Eigen::Vector3d & value)
    {
        output  << YAML::Flow << YAML::BeginSeq
                << value.x() << value.y() << value.z()
                << YAML::EndSeq;
    }

    bool save_results(
        const Eigen::Vector3d & tracker_near_target,
        const Eigen::Vector3d & tracker_far_target,
        double scale,
        std::string & error)
    {
        try
        {
            const std::filesystem::path output_path = result_file_;
            const std::filesystem::path output_directory = output_path.parent_path();
            if (!output_directory.empty())
            {
                std::filesystem::create_directories(output_directory);
            }

            YAML::Emitter output;
            output << YAML::BeginMap;
            output << YAML::Key << "translation_scaling" << YAML::Value;
            output << YAML::BeginMap;

            output << YAML::Key << "target_frame" << YAML::Value << target_frame_;
            output << YAML::Key << "tracker_frame" << YAML::Value << tracker_frame_;
            output << YAML::Key << "scale" << YAML::Value << scale;

            output << YAML::Key << "tracker_to_target_translation_xyz" << YAML::Value;
            emit_vector3(output, tracker_to_target_translation_);

            output << YAML::Key << "tracker_to_target_rotation_xyzw" << YAML::Value;
            output << YAML::Flow << YAML::BeginSeq
                    << tracker_to_target_rotation_.x()
                    << tracker_to_target_rotation_.y()
                    << tracker_to_target_rotation_.z()
                    << tracker_to_target_rotation_.w()
                    << YAML::EndSeq;

            output << YAML::Key << "tracker_near_target" << YAML::Value;
            emit_vector3(output, tracker_near_target);

            output << YAML::Key << "tracker_far_target" << YAML::Value;
            emit_vector3(output, tracker_far_target);

            output << YAML::Key << "robot_near" << YAML::Value;
            emit_vector3(output, robot_near_);

            output << YAML::Key << "robot_far" << YAML::Value;
            emit_vector3(output, robot_far_);

            output << YAML::EndMap;
            output << YAML::EndMap;

            if (!output.good())
            {
                error = output.GetLastError();
                return false;
            }

            const std::filesystem::path temporary_path = output_path.string() + ".tmp";

            {
                std::ofstream file(temporary_path);
                if (!file)
                {
                    error = "Could not open temporary result file.";
                    return false;
                }

                file << output.c_str() << '\n';
                file.flush();

                if (!file)
                {
                    error = "Failed while writing result YAML.";
                    return false;
                }
            }

            std::filesystem::rename(temporary_path, output_path);
            return true;
        }

        catch(const std::exception & ex)
        {
            error = ex.what();
            return false;
        }
    }    

    void run_calibration()
    {
        while (rclcpp::ok())
        {
            try
            {
                wait_for_tracker();

                if (!rclcpp::ok())
                {
                    return;
                }

                std::cerr
                    << "===================================================="
                    << std::endl;
                std::cerr
                    << "                Starting Calibration"
                    << std::endl;
                std::cerr
                    << "===================================================="
                    << std::endl;

                const Eigen::Vector3d tracker_near_target =
                    capture_position(
                        "Hold the tracker close to the body for the near calibration position.");

                std::cerr << "Captured near position in "
                        << target_frame_ << " ["
                        << tracker_near_target.x() << ", "
                        << tracker_near_target.y() << ", "
                        << tracker_near_target.z() << "]"
                        << std::endl;

                const Eigen::Vector3d tracker_far_target =
                    capture_position(
                        "Hold the tracker with a stretched arm forwards.");

                std::cerr << "Captured far position in "
                        << target_frame_ << " ["
                        << tracker_far_target.x() << ", "
                        << tracker_far_target.y() << ", "
                        << tracker_far_target.z() << "]"
                        << std::endl;

                const double scale = calculate_scale(
                    tracker_near_target,
                    tracker_far_target);

                std::string save_error;

                if (!save_results(
                        tracker_near_target,
                        tracker_far_target,
                        scale,
                        save_error))
                {
                    throw std::runtime_error(
                        "Could not save calibration result: " + save_error);
                }

                RCLCPP_INFO(
                    get_logger(),
                    "Calculated scale: %.6f",
                    scale);

                RCLCPP_INFO(
                    get_logger(),
                    "Results saved to: %s",
                    result_file_.c_str());

                rclcpp::shutdown();
                return;
            }
            catch (const RestartCalibrationError & ex)
            {
                if (!rclcpp::ok())
                {
                    return;
                }

                RCLCPP_WARN(
                    get_logger(),
                    "%s",
                    ex.what());

                RCLCPP_WARN(
                    get_logger(),
                    "Please hold the tracker still. Restarting from the near position.");

                // Retry the complete calibration sequence.
                // received_pose_ remains true, so waiting for the first
                // tracker pose again is unnecessary.
                continue;
            }
            catch (const std::exception & ex)
            {
                if (rclcpp::ok())
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Translation scaling calibration failed: %s",
                        ex.what());
                }

                return;
            }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<TranslationScaling>();
        rclcpp::spin(node);
    }
    catch(const std::exception & ex)
    {
        RCLCPP_FATAL(rclcpp::get_logger("translation_scaling"),
            "Could not start node: %s",
            ex.what());
    }

    rclcpp::shutdown();
    return 0;
}