#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robomaster_comm.h"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("robomaster_bridge")
  {
    initialize("can0");
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration(0, 10 * 1e6), std::bind(&MinimalSubscriber::timer_callback, this));
    timer_watchdog_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration(0, 200 * 1e6), std::bind(&MinimalSubscriber::timer_watchdog_callback, this));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    set_twist(msg->linear.x, msg->linear.y, msg->angular.z * 180.0 / 3.1415);
    this->timer_watchdog_->reset();
  }

  void timer_callback(void) const
  {
    run_10ms();
  }

  void timer_watchdog_callback(void) const
  {
    set_twist(0.0, 0.0, 0.0);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_watchdog_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
