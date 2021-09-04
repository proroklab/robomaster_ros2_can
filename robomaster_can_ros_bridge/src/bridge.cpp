#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robomaster_interfaces/srv/led.hpp"

#include "robomaster_comm.h"
using std::placeholders::_1;
using std::placeholders::_2;

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

    led_service_ = this->create_service<robomaster_interfaces::srv::LED>("led", std::bind(&MinimalSubscriber::led_service_callback, this, _1, _2));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    set_twist(msg->linear.x, msg->linear.y, msg->angular.z * 180.0 / 3.1415);
    this->timer_watchdog_->reset();
  }

  void led_service_callback(const std::shared_ptr<robomaster_interfaces::srv::LED::Request> request,
            std::shared_ptr<robomaster_interfaces::srv::LED::Response>      response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nr: %ld g: %ld b: %ld",
                  request->r, request->g, request->b);
    send_cmd_1a_set_led(request->r, request->g, request->b, request->mode, request->speed_up, request->speed_down, 0x3F);
    response->success = true;
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
  rclcpp::Service<robomaster_interfaces::srv::LED>::SharedPtr led_service_;
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
