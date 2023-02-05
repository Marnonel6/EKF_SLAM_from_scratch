#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class odometry : public rclcpp::Node
{
  public:
    odometry()
    : Node("odometry")
    {
        // Parameter descirption
        auto body_id_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto odom_id_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto wheel_left_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto wheel_right_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto wheelradius_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto track_width_des = rcl_interfaces::msg::ParameterDescriptor{};
        body_id_des.description = "The name of the body frame of the robot";
        odom_id_des.description = "The name of the odometry frame";
        wheel_left_des.description = "The name of the left wheel joint";
        wheel_right_des.description = "The name of the right wheel joint.";
        wheelradius_des.description = "The radius of the wheels [m]";
        track_width_des.description = "The distance between the wheels [m]";

        // Declare default parameters values
        declare_parameter("body_id", "", body_id_des);
        declare_parameter("odom_id", "odom", odom_id_des);
        declare_parameter("wheel_left", "", wheel_left_des);
        declare_parameter("wheel_right", "", wheel_right_des);
        declare_parameter("wheelradius", -1.0, wheelradius_des);
        declare_parameter("track_width", -1.0, track_width_des);
        // Get params - Read params from yaml file that is passed in the launch file
        body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
        odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
        wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
        wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
        wheelradius_ = get_parameter("wheelradius").get_parameter_value().get<float>();
        track_width_ = get_parameter("track_width").get_parameter_value().get<float>();


        // Ensures all values are passed via the launch file
        // check_frame_params();

        // Ensures all values are passed via .yaml file
        check_yaml_params();

        // Publishers
        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
                               "odom", 10);

        // Subscribers
        joint_states_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
                        "joint_states", 10, std::bind(&odometry::joint_states_callback,
                                                       this, _1));

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer
        timer_ = create_wall_timer(500ms, std::bind(&odometry::timer_callback, this));
    }

  private:
    // Variables
    std::string body_id_;
    std::string odom_id_;
    std::string wheel_left_;
    std::string wheel_right_;
    float wheelradius_;
    float track_width_;
    sensor_msgs::msg::JointState joint_states_;

    // Create objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// \brief
    /// \param msg
    void joint_states_callback(const sensor_msgs::msg::JointState & msg)
    {
        RCLCPP_INFO(get_logger(), "I heard sensor_data");
    }

    // Ensures all values are passed via the launch file
    void check_frame_params()
    {
        if (body_id_ == "" || wheel_left_ == "" || wheel_right_ == "")
        {
            int err_ = true;
            RCLCPP_ERROR_STREAM(get_logger(), "Missing frame id's! body_id, wheel_left, \
                                                                                     wheel_right");
            throw err_;
        }
    }

    // Ensures all values are passed via .yaml file
    void check_yaml_params()
    {
        if (wheelradius_ == -1.0 || track_width_ == -1.0)
        {
            int err_ = true;
            RCLCPP_ERROR_STREAM(get_logger(), "Missing parameters in diff_params.yaml!");
            throw err_;
        }
    }

    /// \brief Main simulation timer loop
    void timer_callback()
    {
        
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<odometry>());
    } catch (int err_) {}
    rclcpp::shutdown();
    return 0;
}