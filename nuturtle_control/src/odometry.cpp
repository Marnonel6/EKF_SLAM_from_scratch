/// \file
/// \brief The odometry node subscribes to joint_states and publishes to the odom navigation
///        topic. The node handles the odometry calculations of the blue robot.
///
/// PARAMETERS:
///     \param body_id (std::string): The name of the body frame of the robot
///     \param odom_id (std::string): The name of the odometry frame
///     \param wheel_left (std::string): The name of the left wheel joint
///     \param wheel_right (std::string): The name of the right wheel joint
///     \param wheelradius (float): The radius of the wheels [m]
///     \param track_width (float): The distance between the wheels [m]
///
/// PUBLISHES:
///     \param /odom (nav_msgs::msg::Odometry): Odometry publisher
///
/// SUBSCRIBES:
///     \param /joint_states (sensor_msgs::msg::JointState): Subscribes joint states for blue robot
///
/// SERVERS:
///     \param /initial_pose (std_srvs::srv::Empty): Sets initial pose of the turtle
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts blue turtle position

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/diff_drive.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtle_control/srv/initial_config.hpp"

using namespace std::chrono_literals;

/// \brief The odometry node subscribes to joint_states and publishes to the odom navigation
///        topic. It has an initial pose service to set the start position of the robot.
///        The node publishes the location of the blue robot that represents the odometry
///        calcualtions.
///
///  \param body_id_ (std::string): The name of the body frame of the robot
///  \param odom_id_ (std::string): The name of the odometry frame
///  \param wheel_left_ (std::string): The name of the left wheel joint
///  \param wheel_right_ (std::string): The name of the right wheel joint
///  \param wheelradius_ (float): The radius of the wheels [m]
///  \param track_width_ (float): The distance between the wheels [m]

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
        wheel_right_des.description = "The name of the right wheel joint";
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
        check_frame_params();

        // Ensures all values are passed via .yaml file
        check_yaml_params();

        // Update object with params
        turtle_ = turtlelib::DiffDrive{wheelradius_, track_width_};

        // Publishers
        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
                               "odom", 10);

        // Subscribers
        joint_states_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
                        "joint_states", 10, std::bind(&odometry::joint_states_callback,
                                                       this, std::placeholders::_1));

        // Initial pose service
        initial_pose_server_ = create_service<nuturtle_control::srv::InitialConfig>("initial_pose",
        std::bind(&odometry::initial_pose_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

  private:
    // Variables
    std::string body_id_;
    std::string odom_id_;
    std::string wheel_left_;
    std::string wheel_right_;
    float wheelradius_;
    float track_width_;
    turtlelib::Wheel new_wheel_pos_;
    turtlelib::Wheel prev_wheel_pos_{0.0,0.0};
    turtlelib::Twist2D body_twist_;
    turtlelib::DiffDrive turtle_;
    nav_msgs::msg::Odometry odom_;
    tf2::Quaternion q_;
    geometry_msgs::msg::TransformStamped t_;
    sensor_msgs::msg::JointState joint_states_;

    // Create objects
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
    rclcpp::Service<nuturtle_control::srv::InitialConfig>::SharedPtr initial_pose_server_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// \brief Joint states topic callback
    void joint_states_callback(const sensor_msgs::msg::JointState & msg)
    {
        new_wheel_pos_.left = msg.position[0] - prev_wheel_pos_.left;
        new_wheel_pos_.right = msg.position[1] - prev_wheel_pos_.right;
        body_twist_ = turtle_.Twist(new_wheel_pos_);
        turtle_.ForwardKinematics(new_wheel_pos_);
        odometry_pub();
        transform_broadcast();
        prev_wheel_pos_.left = msg.position[0];
        prev_wheel_pos_.right = msg.position[1];
    }

    /// \brief Ensures all values are passed via the launch file
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

    /// \brief Ensures all values are passed via .yaml file
    void check_yaml_params()
    {
        if (wheelradius_ == -1.0 || track_width_ == -1.0)
        {
            int err_ = true;
            RCLCPP_ERROR_STREAM(get_logger(), "Missing parameters in diff_params.yaml!");
            throw err_;
        }
    }

    /// \brief Initial pose service
    void initial_pose_callback(
      nuturtle_control::srv::InitialConfig::Request::SharedPtr request,
      nuturtle_control::srv::InitialConfig::Response::SharedPtr)
    {
      // Set configuration to initial pose
      turtle_ = turtlelib::DiffDrive{wheelradius_, track_width_, {request->x, request->y,\
                                                                  request->theta}};
    }

    /// \brief Broadcasts blue robots position
    void transform_broadcast()
    {
        // Broadcast TF frames
        t_.header.stamp = this->get_clock()->now();
        t_.header.frame_id = odom_id_;
        t_.child_frame_id = body_id_;
        t_.transform.translation.x = turtle_.configuration().x;
        t_.transform.translation.y = turtle_.configuration().y;
        t_.transform.translation.z = 0.0; // Turtle only exists in 2D
        t_.transform.rotation.x = q_.x();
        t_.transform.rotation.y = q_.y();
        t_.transform.rotation.z = q_.z();
        t_.transform.rotation.w = q_.w();
        tf_broadcaster_->sendTransform(t_);
    }

    /// \brief Publishes the odometry to the odom topic
    void odometry_pub()
    {
        // Publish updated odometry
        odom_.header.frame_id = odom_id_;
        odom_.child_frame_id = body_id_;
        odom_.header.stamp = this->get_clock()->now();
        odom_.pose.pose.position.x = turtle_.configuration().x;
        odom_.pose.pose.position.y = turtle_.configuration().y;
        odom_.pose.pose.position.z = 0.0;
        q_.setRPY(0, 0, turtle_.configuration().theta);   // Rotation around z-axis
        odom_.pose.pose.orientation.x = q_.x();
        odom_.pose.pose.orientation.y = q_.y();
        odom_.pose.pose.orientation.z = q_.z();
        odom_.pose.pose.orientation.w = q_.w();
        odom_.twist.twist.linear.x = body_twist_.x;
        odom_.twist.twist.linear.y = body_twist_.y;
        odom_.twist.twist.angular.z = body_twist_.w;
        odom_publisher_->publish(odom_);
    }
};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<odometry>());
    } catch (int err_) {}
    rclcpp::shutdown();
    return 0;
}
