#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "robomaster_interfaces/msg/wheel_speed.hpp"

#include "can_streambuf.hpp"
#include "dds.hpp"
#include "chassis.hpp"

#include <Eigen/Dense>

using namespace Eigen;


using std::placeholders::_1;
using std::placeholders::_2;

using robomaster::dds::metadata;
using robomaster::dds::attitude;
using robomaster::dds::wheel_encoders;
using robomaster::dds::battery;
using robomaster::dds::velocity;

class RoboMasterSensing : public rclcpp::Node
{
public:
    RoboMasterSensing()
        : Node("robomaster_sensing")
        , _can_out_streambuf{"can0", 0x201}
        , _can_out{&_can_out_streambuf}
        , _can_in_streambuf{"can0", 0x202}
        , _can_in{&_can_in_streambuf}
        , _dds{_can_in, _can_out}
    {
        _dds.subscribe(
            std::function<void(const metadata&, const attitude&)>(std::bind(&RoboMasterSensing::cb_attitude, this, _1, _2)),
            50
        );
        _dds.subscribe(
            std::function<void(const metadata&, const wheel_encoders&)>(std::bind(&RoboMasterSensing::cb_wheel_enc, this, _1, _2)),
            50
        );
        _dds.subscribe(
            std::function<void(const metadata&, const velocity&)>(std::bind(&RoboMasterSensing::cb_vel, this, _1, _2)),
            50
        );
        _dds.subscribe(
            std::function<void(const metadata&, const battery&)>(std::bind(&RoboMasterSensing::cb_bat, this, _1, _2)),
            10
        );
        _can_out.flush();

        _speed_publisher = create_publisher<geometry_msgs::msg::TwistStamped>(
                "vel",
                rclcpp::SensorDataQoS()
            );

        _wheel_speed_publisher = create_publisher<robomaster_interfaces::msg::WheelSpeed>(
                "wheel_speed",
                rclcpp::SensorDataQoS()
            );

        _battery_state_publisher = create_publisher<sensor_msgs::msg::BatteryState>(
                "battery_state",
                rclcpp::SensorDataQoS()
            );

        _attitude_publisher = create_publisher<geometry_msgs::msg::QuaternionStamped>(
                "attitude",
                rclcpp::SensorDataQoS()
            );


        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized");
    }

private:

    void cb_attitude(const metadata&, const attitude& attitude)
    {
        Quaternionf q;
        q = AngleAxisf(attitude.roll * 3.141592653589793 / 180.0, Vector3f::UnitX())
            * AngleAxisf(attitude.pitch * 3.141592653589793 / 180.0, Vector3f::UnitY())
            * AngleAxisf(attitude.yaw * 3.141592653589793 / 180.0, Vector3f::UnitZ());

        geometry_msgs::msg::QuaternionStamped attitude_msg{};
        attitude_msg.header.frame_id = "body";
        attitude_msg.header.stamp = get_clock()->now();
        attitude_msg.quaternion.x = q.x();
        attitude_msg.quaternion.y = q.y();
        attitude_msg.quaternion.z = q.z();
        attitude_msg.quaternion.w = q.w();

        _attitude_publisher->publish(attitude_msg);
    }

    void cb_wheel_enc(const metadata&, const wheel_encoders& wheel_encoders)
    {
        robomaster_interfaces::msg::WheelSpeed wheel_speed{};
        wheel_speed.header.frame_id = "body";
        wheel_speed.header.stamp = get_clock()->now();
        wheel_speed.fr = wheel_encoders.rpm[0];
        wheel_speed.fl = -wheel_encoders.rpm[1];
        wheel_speed.rl = -wheel_encoders.rpm[2];
        wheel_speed.rr = wheel_encoders.rpm[3];

        _wheel_speed_publisher->publish(wheel_speed);
    }

    void cb_bat(const metadata&, const battery& battery)
    {
        sensor_msgs::msg::BatteryState bat_msg{};
        bat_msg.header.frame_id = "body";
        bat_msg.header.stamp = get_clock()->now();
        bat_msg.voltage = battery.adc_val / 1000.f;
        bat_msg.temperature = battery.temperature / 10.f;
        bat_msg.current = battery.current / 1000.f;
        bat_msg.percentage = battery.percent / 100.f;

        //bat_msg.charge = ".nan";
        //bat_msg.capacity = ".nan";
        bat_msg.design_capacity = 2.4f;
        bat_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        bat_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        bat_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
        bat_msg.present = true;

        _battery_state_publisher->publish(bat_msg);
    }

    void cb_vel(const metadata&, const velocity& velocity)
    {
        geometry_msgs::msg::TwistStamped twist{};
        twist.header.frame_id = "body";
        twist.header.stamp = get_clock()->now();
        twist.twist.linear.x = velocity.vbx;
        twist.twist.linear.y = velocity.vby;
        _speed_publisher->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _speed_publisher;
    rclcpp::Publisher<robomaster_interfaces::msg::WheelSpeed>::SharedPtr _wheel_speed_publisher;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr _battery_state_publisher;
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr _attitude_publisher;

    can_streambuf _can_out_streambuf;
    std::iostream _can_out;
    can_streambuf _can_in_streambuf;
    std::iostream _can_in;

    robomaster::dds::dds _dds;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboMasterSensing>());
    rclcpp::shutdown();
    return 0;
}
