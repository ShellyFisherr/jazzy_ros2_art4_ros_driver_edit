#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;

class TimerTest : public rclcpp::Node
{
public:
    TimerTest()
        : Node("timer_test"),
          running_(true)
    {
        ros_last_ = std::chrono::steady_clock::now();
        thread_last_ = std::chrono::steady_clock::now();

        ros_timer_ = this->create_wall_timer(
            10ms,
            std::bind(&TimerTest::ros_timer_callback, this));

        worker_ = std::thread(
            &TimerTest::normal_thread_callback,
            this);
    }

    ~TimerTest()
    {
        running_ = false;

        if (worker_.joinable())
            worker_.join();
    }

private:
    void ros_timer_callback()
    {
        auto now = std::chrono::steady_clock::now();

        double dt =
            std::chrono::duration<double>(now - ros_last_).count();

        ros_last_ = now;

        if (dt > ros_max_dt_)
            ros_max_dt_ = dt;

        ros_count_++;

        if (ros_count_ % 100 == 0)
        {
            std::cout
                << "[ROS TIMER] "
                << ros_count_
                << " | dt=" << dt * 1000.0
                << " ms | max=" << ros_max_dt_ * 1000.0
                << " ms"
                << std::endl;
        }
    }

    void normal_thread_callback()
    {
        while (running_)
        {
            std::this_thread::sleep_for(10ms);

            auto now = std::chrono::steady_clock::now();

            double dt =
                std::chrono::duration<double>(
                    now - thread_last_).count();

            thread_last_ = now;

            if (dt > thread_max_dt_)
                thread_max_dt_ = dt;

            thread_count_++;

            if (thread_count_ % 100 == 0)
            {
                std::cout
                    << "[NORMAL THREAD] "
                    << thread_count_
                    << " | dt=" << dt * 1000.0
                    << " ms | max=" << thread_max_dt_ * 1000.0
                    << " ms"
                    << std::endl;
            }
        }
    }

    rclcpp::TimerBase::SharedPtr ros_timer_;

    std::thread worker_;
    std::atomic<bool> running_;

    std::chrono::steady_clock::time_point ros_last_;
    std::chrono::steady_clock::time_point thread_last_;

    uint64_t ros_count_ = 0;
    uint64_t thread_count_ = 0;

    double ros_max_dt_ = 0.0;
    double thread_max_dt_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TimerTest>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}