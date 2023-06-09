/// \file
/// \brief The slam node subscribes to joint_states and publishes to the green/odom navigation
///        topic. The node handles the Data association and SLAM calculations of the green robot.
///
/// PARAMETERS:
///     \param body_id (std::string): The name of the body frame of the robot
///     \param odom_id (std::string): The name of the odometry frame
///     \param wheel_left (std::string): The name of the left wheel joint
///     \param wheel_right (std::string): The name of the right wheel joint
///     \param wheelradius (double): The radius of the wheels [m]
///     \param track_width (double): The distance between the wheels [m]
///     \param obstacles.r (double): Radius of cylindrical obstacles [m]
///     \param obstacles.h (double): Height of cylindrical obstacles [m]
///
/// PUBLISHES:
///     \param /odom (nav_msgs::msg::Odometry): Odometry publisher
///     \param /green/path (nav_msgs::msg::Path): Create the green turtle's nav_msgs/Path for rviz
///                                              visualization
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): SLAM estimate marker obstacles that are
///                                                                displayed in Rviz
///
/// SUBSCRIBES:
///     \param /joint_states (sensor_msgs::msg::JointState): Subscribes joint states for green robot
///     \param /nusim/fake_sensor (visualization_msgs::msg::MarkerArray): Fake sensor circles
///                                                                       MarkerArray as published
///                                                                       by the simulator (NUSIM)
///     \param /landmarks/circle_fit (visualization_msgs::msg::MarkerArray): Fitted circles MarkerArray as
///                                                                 seen by circle fitting algorithm
///
/// SERVERS:
///     \param /initial_pose (std_srvs::srv::Empty): Sets initial pose of the turtle
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts green turtle position
///                                                             relative to odom
///     \param tf_broadcaster_2_ (tf2_ros::TransformBroadcaster): Broadcasts map to odom from SLAM
///                                                               corrections to ensure green turtle
///                                                               is in correct positions

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
#include "visualization_msgs/msg/marker.hpp"

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
///  \param obstacles_r_ (double): Radius of cylindrical obstacles [m]
///  \param obstacles_h_ (double): Height of cylindrical obstacles [m]

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
    auto obstacles_r_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_h_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_landmarks_des = rcl_interfaces::msg::ParameterDescriptor{};
    body_id_des.description = "The name of the body frame of the robot";
    odom_id_des.description = "The name of the odometry frame";
    wheel_left_des.description = "The name of the left wheel joint";
    wheel_right_des.description = "The name of the right wheel joint";
    wheelradius_des.description = "The radius of the wheels [m]";
    track_width_des.description = "The distance between the wheels [m]";
    obstacles_r_des.description = "Radius of cylindrical obstacles [m]";
    obstacles_h_des.description = "Height of cylindrical obstacles [m]";
    use_landmarks_des.description = "Use the clustering and circle fitting node for landmarks";

    // Declare default parameters values
    declare_parameter("body_id", "green/base_footprint", body_id_des);
    declare_parameter("odom_id", "green/odom", odom_id_des);
    declare_parameter("wheel_left", "green/wheel_left_link", wheel_left_des);
    declare_parameter("wheel_right", "green/wheel_right_link", wheel_right_des);
    declare_parameter("wheelradius", -1.0, wheelradius_des);
    declare_parameter("track_width", -1.0, track_width_des);
    declare_parameter("obstacles.r", 0.0, obstacles_r_des);
    declare_parameter("obstacles.h", 0.0, obstacles_h_des);
    declare_parameter("use_landmarks", false, use_landmarks_des);
    // Get params - Read params from yaml file that is passed in the launch file
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    wheelradius_ = get_parameter("wheelradius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    obstacles_h_ = get_parameter("obstacles.h").get_parameter_value().get<double>();
    use_landmarks_ = get_parameter("use_landmarks").get_parameter_value().get<bool>();

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
    obstacles_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

    // Subscribers
    joint_states_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &slam::joint_states_callback,
        this, std::placeholders::_1));
    if (use_landmarks_ == false) { // Use fake sensor markers
      fake_sensor_subscriber_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/nusim/fake_sensor", 10, std::bind(
          &slam::fake_sensor_callback,
          this, std::placeholders::_1));
    } else { // Use landmarks node - Clustering and circle fitting output (Preform data association)
      circle_fitting_subscriber_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/landmarks/circle_fit", 10, std::bind(
          &slam::circle_fitting_callback,
          this, std::placeholders::_1));
    }

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
  double obstacles_r_;    // Size of obstacles
  double obstacles_h_;
  bool use_landmarks_ = false;
  bool Flag_obstacle_seen_ = false;
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
  turtlelib::Robot_configuration green_turtle{};
  turtlelib::Transform2D Tmap_RobotGreen{};
  turtlelib::Transform2D TodomGreen_RobotGreen{};
  turtlelib::Transform2D Tmap_odomGreen{};
  tf2::Quaternion q2_;

  // Create objects
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_turtle_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr circle_fitting_subscriber_;
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
    if (step_ % 100 == 1) {
      green_turtle_NavPath();
    }

    prev_wheel_pos_.left = msg.position[0];
    prev_wheel_pos_.right = msg.position[1];
  }

  /// \brief fake lidar sensor topic callback
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    EKFSlam_.EKFSlam_Predict(
      {turtle_.configuration().theta,
        turtle_.configuration().x, turtle_.configuration().y});

    visualization_msgs::msg::MarkerArray sensed_landmarks = msg;

    for (size_t j = 0; j < sensed_landmarks.markers.size(); j++) {
      if (sensed_landmarks.markers[j].action == visualization_msgs::msg::Marker::ADD) { // Only use landmarks that the sensor currently sees
        EKFSlam_.EKFSlam_Correct(
          sensed_landmarks.markers[j].pose.position.x,
          sensed_landmarks.markers[j].pose.position.y, j);
      }
    }
  }

  /// \brief circle fitting topic callback (Preform data association)
  void circle_fitting_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    EKFSlam_.EKFSlam_Predict(
      {turtle_.configuration().theta,
        turtle_.configuration().x, turtle_.configuration().y});

    visualization_msgs::msg::MarkerArray sensed_landmarks = msg;

    for (size_t j = 0; j < sensed_landmarks.markers.size(); j++) {
      // Preform data association
      size_t index = EKFSlam_.Data_association(
        sensed_landmarks.markers[j].pose.position.x,
        sensed_landmarks.markers[j].pose.position.y);

      // Pass x,y and the ID output from data association to the Correction step of EKF SLAM
      EKFSlam_.EKFSlam_Correct(
        sensed_landmarks.markers[j].pose.position.x,
        sensed_landmarks.markers[j].pose.position.y, index);
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
    green_turtle = EKFSlam_.EKFSlam_config();
    Tmap_RobotGreen = {{green_turtle.x, green_turtle.y}, green_turtle.theta};
    TodomGreen_RobotGreen =
    {{turtle_.configuration().x, turtle_.configuration().y}, turtle_.configuration().theta};
    Tmap_odomGreen = Tmap_RobotGreen * TodomGreen_RobotGreen.inv();

    // Broadcast TF frames
    t2_.header.stamp = get_clock()->now();
    t2_.header.frame_id = "map";
    t2_.child_frame_id = odom_id_;
    t2_.transform.translation.x = Tmap_odomGreen.translation().x;
    t2_.transform.translation.y = Tmap_odomGreen.translation().y;
    t2_.transform.translation.z = 0.0;     // Turtle only exists in 2D
    q2_.setRPY(0, 0, Tmap_odomGreen.rotation());       // Rotation around z-axis
    t2_.transform.rotation.x = q2_.x();
    t2_.transform.rotation.y = q2_.y();
    t2_.transform.rotation.z = q2_.z();
    t2_.transform.rotation.w = q2_.w();
    tf_broadcaster_2_->sendTransform(t2_);

    // SLAM estimate of landmarks
    create_obstacles_array();
  }

  /// \brief Create obstacles MarkerArray as seen by SLAM and publish them to a topic to display them in Rviz
  void create_obstacles_array()
  {

    arma::colvec zai = EKFSlam_.EKFSlam_zai();
    visualization_msgs::msg::MarkerArray obstacles_;

    for (auto i = 3; i < static_cast<int>(zai.size()); i = i + 2) {
      if (zai(i) != 0) {
        visualization_msgs::msg::Marker obstacle_;
        obstacle_.header.frame_id = "map";
        obstacle_.header.stamp = get_clock()->now();
        obstacle_.id = i;
        obstacle_.type = visualization_msgs::msg::Marker::CYLINDER;
        obstacle_.action = visualization_msgs::msg::Marker::ADD;
        obstacle_.pose.position.x = zai(i);
        obstacle_.pose.position.y = zai(i + 1);
        obstacle_.pose.position.z = obstacles_h_ / 2.0;
        obstacle_.pose.orientation.x = 0.0;
        obstacle_.pose.orientation.y = 0.0;
        obstacle_.pose.orientation.z = 0.0;
        obstacle_.pose.orientation.w = 1.0;
        obstacle_.scale.x = obstacles_r_ * 2.0;       // Diameter in x
        obstacle_.scale.y = obstacles_r_ * 2.0;       // Diameter in y
        obstacle_.scale.z = obstacles_h_;             // Height
        obstacle_.color.r = 0.0f;
        obstacle_.color.g = 1.0f;
        obstacle_.color.b = 0.0f;
        obstacle_.color.a = 1.0;
        obstacles_.markers.push_back(obstacle_);
        Flag_obstacle_seen_ = true;
      }
    }
    if (Flag_obstacle_seen_ == true) {
      obstacles_publisher_->publish(obstacles_);
    }
  }

  /// \brief Create the green turtle's nav_msgs/Path
  void green_turtle_NavPath()
  {
    // Update ground truth green turtle path
    green_path_.header.stamp = get_clock()->now();
    green_path_.header.frame_id = "map"; // TODO was green/odom
    // Create new pose stamped
    green_pose_stamped_.header.stamp = get_clock()->now();
    green_pose_stamped_.header.frame_id = "map"; // TODO was green/odom
    green_pose_stamped_.pose.position.x = Tmap_RobotGreen.translation().x; //turtle_.configuration().x;
    green_pose_stamped_.pose.position.y = Tmap_RobotGreen.translation().y; //turtle_.configuration().y;
    green_pose_stamped_.pose.position.z = 0.0;
    green_pose_stamped_.pose.orientation.x = q2_.x(); // q_.x();
    green_pose_stamped_.pose.orientation.y = q2_.y(); // q_.y();
    green_pose_stamped_.pose.orientation.z = q2_.z(); // q_.z();
    green_pose_stamped_.pose.orientation.w = q2_.w(); // q_.w();
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
