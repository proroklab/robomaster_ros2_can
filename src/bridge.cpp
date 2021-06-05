#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("robomaster_bridge")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "test_cmd_vel", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration(0, 10 * 1e6), std::bind(&MinimalSubscriber::timer_callback, this));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear.x);
  }

  void timer_callback(void) const
  {
    RCLCPP_INFO(this->get_logger(), "TIMER");
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
