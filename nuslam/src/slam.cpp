/// \file
/// \brief The slam node subscribes to joint_states and publishes to the green/odom navigation
///        topic. The node handles the SLAM calculations of the green robot.
///
/// PARAMETERS:
///     \param body_id (std::string): The name of the body frame of the robot
///     \param odom_id (std::string): The name of the odometry frame
///     \param wheel_left (std::string): The name of the left wheel joint
///     \param wheel_right (std::string): The name of the right wheel joint
///     \param wheelradius (double): The radius of the wheels [m]
///     \param track_width (double): The distance between the wheels [m]
///
/// PUBLISHES:
///     \param /odom (nav_msgs::msg::Odometry): Odometry publisher
///     \param /green/path (nav_msgs::msg::Path): Create the green turtle's nav_msgs/Path for rviz
///                                              visualization
///
/// SUBSCRIBES:
///     \param /joint_states (sensor_msgs::msg::JointState): Subscribes joint states for green robot
///
/// SERVERS:
///     \param /initial_pose (std_srvs::srv::Empty): Sets initial pose of the turtle
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts green turtle position
///     \param tf_broadcaster_2_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtle_control/srv/initial_config.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

/// \brief The class subscribes to joint_states and publishes to the odom navigation
///        topic. It has an initial pose service to set the start position of the robot.
///        The node publishes the location of the green robot that represents the SLAM
///        calculations.
///
///  \param body_id_ (std::string): The name of the body frame of the robot
///  \param odom_id_ (std::string): The name of the odometry frame
///  \param wheel_left_ (std::string): The name of the left wheel joint
///  \param wheel_right_ (std::string): The name of the right wheel joint
///  \param wheelradius_ (double): The radius of the wheels [m]
///  \param track_width_ (double): The distance between the wheels [m]

class slam : public rclcpp::Node
{
public:
  slam()
  : Node("slam")
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
    declare_parameter("body_id", "green/base_footprint", body_id_des);
    declare_parameter("odom_id", "green/odom", odom_id_des);
    declare_parameter("wheel_left", "green/wheel_left_link", wheel_left_des);
    declare_parameter("wheel_right", "green/wheel_right_link", wheel_right_des);
    declare_parameter("wheelradius", -1.0, wheelradius_des);
    declare_parameter("track_width", -1.0, track_width_des);
    // Get params - Read params from yaml file that is passed in the launch file
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    wheelradius_ = get_parameter("wheelradius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();

    // Ensures all values are passed via the launch file
    check_frame_params();

    // Ensures all values are passed via .yaml file
    check_yaml_params();

    // Update object with params
    turtle_ = turtlelib::DiffDrive{wheelradius_, track_width_};
    // Extended Kalman Filter SLAM object
    EKFSlam_ = turtlelib::EKFSlam{turtle_.configuration()};

    // Publishers
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "green/odom", 10);
    green_turtle_publisher_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);


    // Subscribers
    joint_states_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &slam::joint_states_callback,
        this, std::placeholders::_1));
    fake_sensor_subscriber_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/nusim/fake_sensor", 10, std::bind(
        &slam::fake_sensor_callback,
        this, std::placeholders::_1));

    // Initial pose service
    initial_pose_server_ = create_service<nuturtle_control::srv::InitialConfig>(
      "initial_pose",
      std::bind(
        &slam::initial_pose_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster_2_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  // Variables
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  double wheelradius_;
  double track_width_;
  int step_ = 0;
  turtlelib::Wheel new_wheel_pos_;
  turtlelib::Wheel prev_wheel_pos_{0.0, 0.0};
  turtlelib::Twist2D body_twist_;
  turtlelib::DiffDrive turtle_;
  turtlelib::EKFSlam EKFSlam_{};
  nav_msgs::msg::Odometry odom_;
  tf2::Quaternion q_;
  geometry_msgs::msg::TransformStamped t_;
  geometry_msgs::msg::TransformStamped t2_;
  sensor_msgs::msg::JointState joint_states_;
  geometry_msgs::msg::PoseStamped green_pose_stamped_;
  nav_msgs::msg::Path green_path_;

  // Create objects
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_turtle_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_subscriber_;
  rclcpp::Service<nuturtle_control::srv::InitialConfig>::SharedPtr initial_pose_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_2_;

  /// \brief Joint states topic callback
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    new_wheel_pos_.left = msg.position[0] - prev_wheel_pos_.left;
    new_wheel_pos_.right = msg.position[1] - prev_wheel_pos_.right;
    body_twist_ = turtle_.Twist(new_wheel_pos_);
    turtle_.ForwardKinematics(new_wheel_pos_);
    q_.setRPY(0, 0, turtle_.configuration().theta);       // Rotation around z-axis
    odometry_pub();
    transform_broadcast();
    transform_broadcast_map_odom(); // EKFSlam Update

    step_++;
    if (step_%100 == 1)
    {
        green_turtle_NavPath();
    }

    prev_wheel_pos_.left = msg.position[0];
    prev_wheel_pos_.right = msg.position[1];
  }

  /// \brief Joint states topic callback
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    // EKFSlam_.EKFSlam_Predict(body_twist_);
    // RCLCPP_ERROR_STREAM(get_logger(), "Turtle -> x: " << turtle_.configuration().x << " y: " << turtle_.configuration().y  << " theta: " << turtle_.configuration().theta);
    EKFSlam_.EKFSlam_Predict({turtle_.configuration().theta,turtle_.configuration().x,turtle_.configuration().y});

    visualization_msgs::msg::MarkerArray sensed_landmarks = msg;

    for (size_t j = 0; j < sensed_landmarks.markers.size(); j++)
    {
        if (sensed_landmarks.markers[j].action < 2) // Only use landmarks that the sensor currently sees
        {
            // if (auto search = EKFSlam_.seen_landmarks.find(j); search != EKFSlam_.seen_landmarks.end()){
            //     RCLCPP_ERROR_STREAM(get_logger(), "seen_landmarks " << *search);
            // }
            // else{ RCLCPP_ERROR_STREAM(get_logger(), "_______________NO LANDMARKS");}

            EKFSlam_.EKFSlam_Correct(sensed_landmarks.markers[j].pose.position.x,sensed_landmarks.markers[j].pose.position.y,j);
            // RCLCPP_ERROR_STREAM(get_logger(), "Hj*estimate*Hj.T " << EKFSlam_.Hj*EKFSlam_.covariance_estimate*EKFSlam_.Hj.t());
        }
    }
  }

  /// \brief Ensures all values are passed via the launch file
  void check_frame_params()
  {
    if (body_id_ == "" || wheel_left_ == "" || wheel_right_ == "") {
      throw std::runtime_error("Missing frame id's! body_id, wheel_left, wheel_right");
    }
  }

  /// \brief Ensures all values are passed via .yaml file
  void check_yaml_params()
  {
    if (wheelradius_ == -1.0 || track_width_ == -1.0) {
      throw std::runtime_error("Missing parameters in diff_params.yaml!");
    }
  }

  /// \brief Initial pose service
  void initial_pose_callback(
    nuturtle_control::srv::InitialConfig::Request::SharedPtr request,
    nuturtle_control::srv::InitialConfig::Response::SharedPtr)
  {
    // Set configuration to initial pose
    turtle_ = turtlelib::DiffDrive{wheelradius_, track_width_, {request->x, request->y, \
        request->theta}};
  }

  /// \brief Broadcasts green robots position
  void transform_broadcast()
  {
    // Broadcast TF frames
    t_.header.stamp = get_clock()->now();
    t_.header.frame_id = odom_id_;
    t_.child_frame_id = body_id_;
    t_.transform.translation.x = turtle_.configuration().x;
    t_.transform.translation.y = turtle_.configuration().y;
    t_.transform.translation.z = 0.0;     // Turtle only exists in 2D
    t_.transform.rotation.x = q_.x();
    t_.transform.rotation.y = q_.y();
    t_.transform.rotation.z = q_.z();
    t_.transform.rotation.w = q_.w();
    tf_broadcaster_->sendTransform(t_);
  }

  /// \brief Broadcasts green robots position
  void transform_broadcast_map_odom()
  {
    turtlelib::Robot_configuration green_turtle = EKFSlam_.EKFSlam_config();
    // turtlelib::Robot_configuration green_turtle = EKFSlam_.EKFSlam_config_predicted();

    turtlelib::Transform2D Tmap_RobotGreen{{green_turtle.x, green_turtle.y},green_turtle.theta};
    turtlelib::Transform2D TodomGreen_RobotGreen{{turtle_.configuration().x,turtle_.configuration().y},turtle_.configuration().theta};
    turtlelib::Transform2D Tmap_odomGreen{};

    Tmap_odomGreen = Tmap_RobotGreen*TodomGreen_RobotGreen.inv();

    // RCLCPP_ERROR_STREAM(get_logger(), "green_turtle -> x: " << green_turtle.x << " y: " << green_turtle.y  << " theta: " << green_turtle.theta);
    
    // RCLCPP_ERROR_STREAM(get_logger(), "Tmap_odomGreen -> rot: " << Tmap_odomGreen.rotation()<< " trans: "<<Tmap_odomGreen.translation().x<<" "<<Tmap_odomGreen.translation().y);
    // Broadcast TF frames
    t2_.header.stamp = get_clock()->now();
    t2_.header.frame_id = "map";
    t2_.child_frame_id = odom_id_;
    t2_.transform.translation.x = Tmap_odomGreen.translation().x;
    t2_.transform.translation.y = Tmap_odomGreen.translation().y;
    // t2_.transform.translation.x = -(green_turtle.x - turtle_.configuration().x);
    // t2_.transform.translation.y = -(green_turtle.y - turtle_.configuration().y);
    t2_.transform.translation.z = 0.0;     // Turtle only exists in 2D
    tf2::Quaternion q2_;
    q2_.setRPY(0, 0, Tmap_odomGreen.rotation());       // Rotation around z-axis
    // // q_.setRPY(0, 0, turtlelib::normalize_angle(-(green_turtle.theta - turtle_.configuration().theta)));       // Rotation around z-axis
    t2_.transform.rotation.x = q2_.x();
    t2_.transform.rotation.y = q2_.y();
    t2_.transform.rotation.z = q2_.z();
    t2_.transform.rotation.w = q2_.w();
    tf_broadcaster_2_->sendTransform(t2_);
  }

  /// \brief Create the green turtle's nav_msgs/Path
  void green_turtle_NavPath()
  {
    // Update ground truth green turtle path
    green_path_.header.stamp = get_clock()->now();
    green_path_.header.frame_id = "nusim/world";
    // Create new pose stamped
    green_pose_stamped_.header.stamp = get_clock()->now();
    green_pose_stamped_.header.frame_id = "nusim/world";
    green_pose_stamped_.pose.position.x = turtle_.configuration().x;
    green_pose_stamped_.pose.position.y = turtle_.configuration().y;
    green_pose_stamped_.pose.position.z = 0.0;
    green_pose_stamped_.pose.orientation.x = q_.x();
    green_pose_stamped_.pose.orientation.y = q_.y();
    green_pose_stamped_.pose.orientation.z = q_.z();
    green_pose_stamped_.pose.orientation.w = q_.w();
    // Append pose stamped
    green_path_.poses.push_back(green_pose_stamped_);
    green_turtle_publisher_->publish(green_path_);
  }

  /// \brief Publishes the odometry to the odom topic
  void odometry_pub()
  {
    // Publish updated odometry
    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;
    odom_.header.stamp = get_clock()->now();
    odom_.pose.pose.position.x = turtle_.configuration().x;
    odom_.pose.pose.position.y = turtle_.configuration().y;
    odom_.pose.pose.position.z = 0.0;
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
  rclcpp::spin(std::make_shared<slam>());
  rclcpp::shutdown();
  return 0;
}
