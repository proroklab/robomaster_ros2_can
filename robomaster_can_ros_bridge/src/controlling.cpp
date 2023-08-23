#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robomaster_msgs/srv/led.hpp"
#include "robomaster_msgs/msg/wheel_speed.hpp"
#include "emergency_stop_msgs/srv/emergency_stop.hpp"

#include "chassis.hpp"
#include "led.hpp"
#include "can_streambuf.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class RoboMasterControlling : public rclcpp::Node
{
public:
    RoboMasterControlling()
        : Node("robomaster_controlling")
        , last_led_request_{}
        , emerg_led_request_{}
        , emergency_stopped_{false}
        , chassis_workmode_enabled_{false}
        , _can_streambuf{"can0", 0x201}
        , _can_iostream{&_can_streambuf}
        , _rm_chassis{_can_iostream}
        , _rm_led{_can_iostream}
    {
        speed_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel",
                rclcpp::SensorDataQoS(),
                std::bind(&RoboMasterControlling::topic_callback_speed, this, _1)
            );
        wheel_speed_subscription_ = create_subscription<robomaster_msgs::msg::WheelSpeed>(
                "cmd_wheels",
                rclcpp::SensorDataQoS(),
                std::bind(&RoboMasterControlling::topic_callback_wheel_speed, this, _1)
            );
        timer_heartbeat_ = rclcpp::create_timer(
                this,
                get_clock(),
                rclcpp::Duration(0, 10 * 1e6),
                std::bind(&robomaster::command::chassis::send_heartbeat, &_rm_chassis)
            );
        timer_watchdog_ = rclcpp::create_timer(
                this,
                get_clock(),
                rclcpp::Duration(0, 200 * 1e6),
                std::bind(&RoboMasterControlling::timer_watchdog_callback, this)
            );

        led_service_ = create_service<robomaster_msgs::srv::LED>(
                "led",
                std::bind(&RoboMasterControlling::led_service_callback, this, _1, _2)
            );
        emergency_stop_service_ = create_service<emergency_stop_msgs::srv::EmergencyStop>(
                "emergency_stop",
                std::bind(&RoboMasterControlling::emergency_stop_service_callback, this, _1, _2)
            );

        emerg_led_request_.r = 255;
        emerg_led_request_.mode = 1;

        last_led_request_.mode = 1;
        last_led_request_.r = 127;
        last_led_request_.g = 127;
        last_led_request_.b = 127;
        update_leds(last_led_request_);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized");
    }

private:
    void topic_callback_speed(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!emergency_stopped_)
        {
            _rm_chassis.send_speed(msg->linear.x, msg->linear.y, msg->angular.z * 180.0 / 3.1415);
        }

        reset_watchdog();
    }

    void topic_callback_wheel_speed(const robomaster_msgs::msg::WheelSpeed::SharedPtr msg)
    {
        if (!emergency_stopped_)
        {
            _rm_chassis.send_wheel_speed(msg->fr, msg->fl, msg->rl, msg->rr);
        }

        reset_watchdog();
    }

    void led_service_callback(const std::shared_ptr<robomaster_msgs::srv::LED::Request> request,
        std::shared_ptr<robomaster_msgs::srv::LED::Response>      response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nr: %d g: %d b: %d",
            request->r, request->g, request->b);

        last_led_request_ = *request;
        update_leds(last_led_request_);
        response->success = true;
    }

    void emergency_stop_service_callback(const std::shared_ptr<emergency_stop_msgs::srv::EmergencyStop::Request> request,
        std::shared_ptr<emergency_stop_msgs::srv::EmergencyStop::Response>      response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency stop request %d", request->stop);
        emergency_stopped_ = request->stop;

        if (request->stop)
        {
            _rm_chassis.send_wheel_speed(0, 0, 0, 0);
            update_leds(emerg_led_request_);
        }
        else
        {
            update_leds(last_led_request_);
        }

        response->success = true;
    }

    void timer_watchdog_callback(void)
    {
        if (chassis_workmode_enabled_)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Watchdog triggered");
            _rm_chassis.send_workmode(0);
            chassis_workmode_enabled_ = false;
        }

        _rm_chassis.send_wheel_speed(0, 0, 0, 0);
    }

    void update_leds(const robomaster_msgs::srv::LED::Request& request)
    {
        _rm_led.send_led(request.mode, request.r, request.g, request.b, request.speed_up, request.speed_down, 0x3F);
    }

    void reset_watchdog()
    {
        if (!chassis_workmode_enabled_)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Watchdog reset");
            _rm_chassis.send_workmode(1);
            chassis_workmode_enabled_ = true;
        }

        timer_watchdog_->reset();
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed_subscription_;
    rclcpp::Subscription<robomaster_msgs::msg::WheelSpeed>::SharedPtr wheel_speed_subscription_;
    rclcpp::Service<robomaster_msgs::srv::LED>::SharedPtr led_service_;
    rclcpp::Service<emergency_stop_msgs::srv::EmergencyStop>::SharedPtr emergency_stop_service_;
    rclcpp::TimerBase::SharedPtr timer_heartbeat_;
    rclcpp::TimerBase::SharedPtr timer_watchdog_;
    robomaster_msgs::srv::LED::Request last_led_request_;
    robomaster_msgs::srv::LED::Request emerg_led_request_;
    bool emergency_stopped_;
    bool chassis_workmode_enabled_;

    can_streambuf _can_streambuf;
    std::iostream _can_iostream;
    robomaster::command::chassis _rm_chassis;
    robomaster::command::led _rm_led;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboMasterControlling>());
    rclcpp::shutdown();
    return 0;
}
