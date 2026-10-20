#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

class TimerTest : public rclcpp::Node
{
public:
  TimerTest()
  : Node("timer_test"),
    timer_count_(0),
    max_dt_(0.0),
    last_time_(std::chrono::steady_clock::now())
  {
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&TimerTest::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Starting 100 Hz timer test");
  }

private:
  void timer_callback()
  {
    const auto now = std::chrono::steady_clock::now();

    const double dt =
      std::chrono::duration<double>(
        now - last_time_).count();

    last_time_ = now;

    if (dt > max_dt_) {
      max_dt_ = dt;
    }

    timer_count_++;

    if (timer_count_ % 100 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "TIMER: %lu | DT: %.3f ms | MAX_DT: %.3f ms",
        timer_count_,
        dt * 1000.0,
        max_dt_ * 1000.0);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;

  uint64_t timer_count_;
  double max_dt_;

  std::chrono::steady_clock::time_point last_time_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TimerTest>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}