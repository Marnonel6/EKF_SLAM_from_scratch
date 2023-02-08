#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/// \brief turtle_control
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
        declare_parameter("wheelradius", -1.0, wheelradius_des);
        declare_parameter("track_width", -1.0, track_width_des);
        declare_parameter("motor_cmd_max", -1.0, motor_cmd_max_des);
        declare_parameter("motor_cmd_per_rad_sec", -1.0, motor_cmd_per_rad_sec_des);
        declare_parameter("encoder_ticks_per_rad", -1.0, encoder_ticks_per_rad_des);
        declare_parameter("collision_radius", -1.0, collision_radius_des);
        // Get params - Read params from yaml file that is passed in the launch file
        wheelradius_ = get_parameter("wheelradius").get_parameter_value().get<float>();
        track_width_ = get_parameter("track_width").get_parameter_value().get<float>();
        motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<float>();
        motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<float>();
        encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<float>();
        collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<float>();

        // Ensures all values are passed via .yaml file
        check_yaml_params();

        // Create Diff Drive Object
        turtle_ = turtlelib::DiffDrive(wheelradius_, track_width_);

        // Publishers
        wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
                               "wheel_cmd", 10);
        joint_states_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
                               "joint_states", 10);

        // Subscribers
        cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
                        "cmd_vel", 10, std::bind(&turtle_control::cmd_vel_callback, this, _1));
        sensor_data_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
                        "sensor_data", 10, std::bind(&turtle_control::sensor_data_callback,
                                                     this, _1));

        // Timer
        timer_ = create_wall_timer(500ms, std::bind(&turtle_control::timer_callback, this));
    }

  private:
    // Variables
    float wheelradius_;
    float track_width_;
    float motor_cmd_max_;
    float motor_cmd_per_rad_sec_;
    float encoder_ticks_per_rad_;
    float collision_radius_;
    float prev_encoder_stamp_ = -1.0;
    turtlelib::Twist2D body_twist_;
    turtlelib::WheelVelocities wheel_vel_;
    turtlelib::DiffDrive turtle_;
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;
    sensor_msgs::msg::JointState joint_states_;

    // Create objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscriber_;

    /// \brief
    /// \param msg
    void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
    {
        body_twist_.w = msg.angular.z;
        body_twist_.x = msg.linear.x;
        body_twist_.y = msg.linear.y;

        // Create Diff Drive Object
        // turtlelib::DiffDrive turtle_(wheelradius_, track_width_);
        // Perform Inverse kinematics to get the wheel velocities from the twist
        wheel_vel_ = turtle_.InverseKinematics(body_twist_);
        // Convert rad/sec to ticks
        wheel_cmd_.left_velocity = wheel_vel_.left/motor_cmd_per_rad_sec_;
        wheel_cmd_.right_velocity = wheel_vel_.right/motor_cmd_per_rad_sec_;

        // Limit max wheel command speed and publish wheel command
        wheel_cmd_.left_velocity = limit_Max(wheel_cmd_.left_velocity);
        wheel_cmd_.right_velocity = limit_Max(wheel_cmd_.right_velocity);
        wheel_cmd_publisher_->publish(wheel_cmd_);
    }

    /// \brief
    /// \param wheel_vel
    double limit_Max(double wheel_vel)
    {
        if (wheel_vel > motor_cmd_max_)
        {
            return motor_cmd_max_;
        }
        else if (wheel_vel < -motor_cmd_max_)
        {
            return -motor_cmd_max_;
        }
        else
        {
            return wheel_vel;
        } 
    }

    /// \brief
    /// \param msg
    void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
    {
        joint_states_.header.stamp = msg.stamp;
        joint_states_.name = {"wheel_left_joint", "wheel_right_joint"};
        if (prev_encoder_stamp_ == -1.0)
        {
            joint_states_.position = {0.0, 0.0};
            joint_states_.velocity = {0.0, 0.0};
        }
        else
        {
            // Change in wheel angle from encoder ticks
            joint_states_.position = {msg.left_encoder/encoder_ticks_per_rad_,
                                      msg.right_encoder/encoder_ticks_per_rad_};
            float passed_time = msg.stamp.sec + msg.stamp.nanosec*1e-9 - prev_encoder_stamp_;
            // Encoder ticks to rad/s
            joint_states_.velocity = {joint_states_.position.at(0)/passed_time,
                                     joint_states_.position.at(1)/passed_time};
        }
        prev_encoder_stamp_ = msg.stamp.sec + msg.stamp.nanosec*1e-9;
        joint_states_publisher_->publish(joint_states_);
    }

    // Ensures all values are passed via .yaml file
    void check_yaml_params()
    {
        if (wheelradius_ == -1.0 || track_width_ == -1.0 || motor_cmd_max_ == -1.0 ||
            motor_cmd_per_rad_sec_ == -1.0 || encoder_ticks_per_rad_ == -1.0 ||
            collision_radius_ == -1.0)
        {
            int err_ = true;
            RCLCPP_ERROR_STREAM(get_logger(), "Missing parameters in diff_params.yaml!");
            throw err_;
        }
    }

    /// \brief Main timer loop
    void timer_callback()
    {
        // wheel_cmd_publisher_->publish(wheel_cmd_);
        // joint_states_publisher_->publish(joint_states_);

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<turtle_control>());
    } catch (int err_) {}
    rclcpp::shutdown();
    return 0;
}