#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robomaster_interfaces/srv/led.hpp"
#include "ctrl_msgs/srv/emergency_stop.hpp"

#include "robomaster_comm.h"
using std::placeholders::_1;
using std::placeholders::_2;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("robomaster_bridge"),
    last_led_request_{},
    emerg_led_request_{},
    emergency_stopped_{false}
  {
    emerg_led_request_.r = 255;
    emerg_led_request_.mode = 1;
    last_led_request_.mode = 1;

    initialize("can0");
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration(0, 10 * 1e6), std::bind(&MinimalSubscriber::timer_callback, this));
    timer_watchdog_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration(0, 200 * 1e6), std::bind(&MinimalSubscriber::timer_watchdog_callback, this));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized");
    led_service_ = this->create_service<robomaster_interfaces::srv::LED>("led", std::bind(&MinimalSubscriber::led_service_callback, this, _1, _2));
    emergency_stop_service_ = this->create_service<ctrl_msgs::srv::EmergencyStop>("emergency_stop", std::bind(&MinimalSubscriber::emergency_stop_service_callback, this, _1, _2));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    if (emergency_stopped_)
    {
      set_twist(0.0, 0.0, 0.0);
    }
    else
    {
      set_twist(msg->linear.x, msg->linear.y, msg->angular.z * 180.0 / 3.1415);
    }

    this->timer_watchdog_->reset();
  }

  void led_service_callback(const std::shared_ptr<robomaster_interfaces::srv::LED::Request> request,
            std::shared_ptr<robomaster_interfaces::srv::LED::Response>      response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nr: %ld g: %ld b: %ld",
                  request->r, request->g, request->b);

    last_led_request_ = *request;
    update_leds(last_led_request_);
    response->success = true;
  }

  void emergency_stop_service_callback(const std::shared_ptr<ctrl_msgs::srv::EmergencyStop::Request> request,
            std::shared_ptr<ctrl_msgs::srv::EmergencyStop::Response>      response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency stop request %d", request->stop);
    emergency_stopped_ = request->stop;
    if (request->stop)
    {
      set_twist(0.0, 0.0, 0.0);
      update_leds(emerg_led_request_);
    }
    else
    {
      update_leds(last_led_request_);
    }
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

  void update_leds(const robomaster_interfaces::srv::LED::Request& request)
  {
    // send LED command multiple times as it sometimes doesn't make it through...
    for(int i = 0; i < 2; i++)
        send_cmd_1a_set_led(request.r, request.g, request.b, request.mode, request.speed_up, request.speed_down, 0x3F);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Service<robomaster_interfaces::srv::LED>::SharedPtr led_service_;
  rclcpp::Service<ctrl_msgs::srv::EmergencyStop>::SharedPtr emergency_stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_watchdog_;
  robomaster_interfaces::srv::LED::Request last_led_request_;
  robomaster_interfaces::srv::LED::Request emerg_led_request_;
  bool emergency_stopped_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
