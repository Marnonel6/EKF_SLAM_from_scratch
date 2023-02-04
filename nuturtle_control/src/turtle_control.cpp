#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
// #include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class turtle_control : public rclcpp::Node
{
  public:
    turtle_control()
    : Node("turtle_control")
    {
        // Parameter descirption
        auto wheelradius_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto track_width_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto motor_cmd_max_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto motor_cmd_per_rad_sec_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto encoder_ticks_per_rad_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto collision_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
        wheelradius_des.description = "The radius of the wheels [m]";
        track_width_des.description = "The distance between the wheels [m]";
        motor_cmd_max_des.description = "The motors are provided commands in the interval \
                                         [-motor_cmd_max, motor_cmd_max]";
        motor_cmd_per_rad_sec_des.description = "Each motor command 'tick' is X [radians/sec]";
        encoder_ticks_per_rad_des.description = "The number of encoder 'ticks' per radian \
                                                 [ticks/rad]";
        collision_radius_des.description = "This is a simplified geometry used for collision \
                                            detection [m]";

        // Declare default parameters values
        declare_parameter("wheelradius", 0.0, wheelradius_des);
        declare_parameter("track_width", 0.0, track_width_des);
        declare_parameter("motor_cmd_max", 0.0, motor_cmd_max_des);
        declare_parameter("motor_cmd_per_rad_sec", 0.0, motor_cmd_per_rad_sec_des);
        declare_parameter("encoder_ticks_per_rad", 0.0, encoder_ticks_per_rad_des);
        declare_parameter("collision_radius", 0.0, collision_radius_des);
        // Get params - Read params from yaml file that is passed in the launch file
        wheelradius_ = get_parameter("wheelradius").get_parameter_value().get<float>();
        track_width_ = get_parameter("track_width").get_parameter_value().get<float>();
        motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<double>();
        motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<float>();
        encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<float>();
        collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<float>();

        // Publishers
        wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
                               "wheel_cmd", 10);

        // Subscribers
        cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
                        "cmd_vel", 10, std::bind(&turtle_control::cmd_vel_callback, this, _1));

        // Timer
        timer_ = create_wall_timer(500ms, std::bind(&turtle_control::timer_callback, this));
    }

  private:
    // Variables
    float wheelradius_ = 0.0;
    float track_width_ = 0.0;
    double motor_cmd_max_ = 0;
    float motor_cmd_per_rad_sec_ = 0.0;
    float encoder_ticks_per_rad_ = 0.0;
    float collision_radius_ = 0.0;
    geometry_msgs::msg::Twist body_twist_;
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;

    // Create objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    /// \brief
    /// \param msg
    void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
    {
        body_twist_ = msg; //  Make this a Twist2D from turtlelib
        RCLCPP_INFO(get_logger(), "I heard data");
    }

    /// \brief Main simulation timer loop
    void timer_callback()
    {
        // auto message = geometry_msgs::msg::Twist();
        // message.linear.x = 1;
        // RCLCPP_INFO(get_logger(), "Publishing: '%f'", message.linear.x);
        // cmd_vel_publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtle_control>());
    rclcpp::shutdown();
    return 0;
}