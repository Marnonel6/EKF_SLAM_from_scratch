/// \file
/// \brief The nusim node is a simulation and visualization tool for the turtlebot3 robots.
///        It uses rviz2 for visualization and provides a simulated environment. The package
///        creates stationary walls and obstacles and track the position of a red robot.
///
/// PARAMETERS:
///     \param rate (int): Timer callback frequency [Hz]
///     \param x0 (float): Initial x coordinate of the robot [m]
///     \param y0 (float): Initial y coordinate of the robot [m]
///     \param theta0 (float): Initial theta angle of the robot [radians]
///     \param obstacles.x (std::vector<double>): Vector of x coordinates for each obstacle [m]
///     \param obstacles.y (std::vector<double>): Vector of y coordinates for each obstacle [m]
///     \param obstacles.r (float): Radius of cylindrical obstacles [m]
///     \param obstacles.h (float): Height of cylindrical obstacles [m]
///     \param walls.x_lenght (float): Inner lenght of walls in x direction [m]
///     \param walls.y_lenght (float): Inner lenght of walls in y direction [m]
///     \param walls.h (float): Walls height [m]
///     \param walls.w (float): Walls width [m]
///     \param motor_cmd_per_rad_sec (float): Motor command to rad/s conversion factor
///     \param encoder_ticks_per_rad (float): Encoder ticks to radians conversion factor
///     \param input_noise (float): Noise added to input signals from turtlebot
///     \param slip_fraction (float): Wheel slippage factor for turtlebot
///
/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): Current simulation timestep
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): Marker obstacles that are
///                                                                displayed in Rviz
///     \param ~/walls (visualization_msgs::msg::MarkerArray): Marker walls that are
///                                                            displayed in Rviz
///     \param /red/sensor_data (nuturtlebot_msgs::msg::SensorData): This is the wheel encoder
///                                                                  output in position ticks
///     \param /red/path (nav_msgs::msg::Path): Create the red turtle's nav_msgs/Path for rviz
///                                             visualization
///
/// SUBSCRIBES:
///     \param /red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Wheel command velocity in
///                                                                   ticks
///
/// SERVERS:
///     \param ~/reset (std_srvs::srv::Empty): Resets simulation to initial state
///     \param ~/teleport (nusim::srv::Teleport): Teleport robot to a specific pose
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts red turtle position

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

/// \brief This class publishes the current timestep of the simulation, obstacles and walls that
///        appear in Rviz as markers. The class has a timer_callback to continually update the
///        simulation at each timestep. The reset service resets the simulation to the initial
///        state thus restarting the simulation. A teleport service is available to teleport a
///        turtlebot to any pose. A broadcaster broadcasts the robots TF frames to a topic for
///        visualization in Rviz. The simulation operates in a loop, updating the state of the
///        world, publishing messages that provides state information simulating a real robot,
///        and processing service/subscriber callbacks for commands for the next time step. The
///        loop runs at a fixed frequency until termination.
///
///  \param rate (int): Timer callback frequency [Hz]
///  \param x0_ (float): Initial x coordinate of the robot [m]
///  \param y0_ (float): Initial y coordinate of the robot [m]
///  \param theta0_ (float): Initial theta angle of the robot [radians]
///  \param x_ (float): Current x coordinate of the robot [m]
///  \param y_ (float): Current y coordinate of the robot [m]
///  \param theta_ (float): Current theta angle of the robot [radians]
///  \param obstacles_x_ (std::vector<double>): Vector of x coordinates for each obstacle [m]
///  \param obstacles_y_ (std::vector<double>): Vector of y coordinates for each obstacle [m]
///  \param obstacles_r_ (float): Radius of cylindrical obstacles [m]
///  \param obstacles_h_ (float): Height of cylindrical obstacles [m]
///  \param walls_x_lenght_ (float): Inner lenght of walls in x direction [m]
///  \param walls_y_lenght_ (float): Inner lenght of walls in y direction [m]
///  \param walls_h_(float): Walls height [m]
///  \param walls_w_(float): Walls width [m]
///  \param wheelradius_ (float): The radius of the wheels [m]
///  \param track_width_ (float): The distance between the wheels [m]
///  \param motor_cmd_per_rad_sec_ (float): Motor command to rad/s conversion factor
///  \param encoder_ticks_per_rad_ (float): Encoder ticks to radians conversion factor
///  \param input_noise_ (float): Noise added to input signals from turtlebot
///  \param slip_fraction_ (float): Wheel slippage factor for turtlebot

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("Nusim"), timestep_(0)
  {
    // Parameter descirption
    auto rate_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto x0_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto y0_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto theta0_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_x_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_y_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_r_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_h_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto walls_x_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto walls_y_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto walls_l_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto walls_h_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto walls_w_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto wheelradius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto track_width_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto encoder_ticks_per_rad_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto motor_cmd_per_rad_sec_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto input_noise_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto slip_fraction_des = rcl_interfaces::msg::ParameterDescriptor{};
    rate_des.description = "Timer callback frequency [Hz]";
    x0_des.description = "Initial x coordinate of the robot [m]";
    y0_des.description = "Initial y coordinate of the robot [m]";
    theta0_des.description = "Initial theta angle of the robot [radians]";
    obstacles_x_des.description = "Vector of x coordinates for each obstacle [m]";
    obstacles_y_des.description = "Vector of y coordinates for each obstacle [m]";
    obstacles_r_des.description = "Radius of cylindrical obstacles [m]";
    obstacles_h_des.description = "Height of cylindrical obstacles [m]";
    walls_x_des.description = "Vector of x coordinates for each wall [m]";
    walls_y_des.description = "Vector of y coordinates for each wall [m]";
    walls_h_des.description = "Height of rectangular wall [m]";
    walls_w_des.description = "Width of rectangular wall [m]";
    wheelradius_des.description = "The radius of the wheels [m]";
    track_width_des.description = "The distance between the wheels [m]";
    encoder_ticks_per_rad_des.description =
      "The number of encoder 'ticks' per radian \
                                               [ticks/rad]";
    motor_cmd_per_rad_sec_des.description =
      "Each motor command 'tick' is 0.024 rad/sec \
                                               [tick/(rad/sec)]";
    input_noise_des.decription = "Noise added to input signals from turtlebot";
    slip_fraction_des.decription = "Wheel slippage factor for turtlebot";

    // Declare default parameters values
    declare_parameter("rate", 200, rate_des);     // Hz for timer_callback
    declare_parameter("x0", 0.0, x0_des);
    declare_parameter("y0", 0.0, y0_des);
    declare_parameter("theta0", 0.0, theta0_des);
    declare_parameter("obstacles.x", std::vector<double>{}, obstacles_x_des);
    declare_parameter("obstacles.y", std::vector<double>{}, obstacles_y_des);
    declare_parameter("obstacles.r", 0.0, obstacles_r_des);
    declare_parameter("obstacles.h", 0.0, obstacles_h_des);
    declare_parameter("walls.x_lenght", 0.0, walls_x_des);
    declare_parameter("walls.y_lenght", 0.0, walls_y_des);
    declare_parameter("walls.h", 0.0, walls_h_des);
    declare_parameter("walls.w", 0.0, walls_w_des);
    declare_parameter("wheelradius", -1.0, wheelradius_des);
    declare_parameter("track_width", -1.0, track_width_des);
    declare_parameter("encoder_ticks_per_rad", -1.0, encoder_ticks_per_rad_des);
    declare_parameter("motor_cmd_per_rad_sec", -1.0, motor_cmd_per_rad_sec_des);
    declare_parameter("input_noise", 0.0, input_noise_des);
    declare_parameter("slip_fraction", 0.0, slip_fraction_des);

    // Get params - Read params from yaml file that is passed in the launch file
    int rate = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<float>();
    y0_ = get_parameter("y0").get_parameter_value().get<float>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<float>();
    obstacles_x_ = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacles_y_ = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<float>();
    obstacles_h_ = get_parameter("obstacles.h").get_parameter_value().get<float>();
    walls_x_ = get_parameter("walls.x_lenght").get_parameter_value().get<float>();
    walls_y_ = get_parameter("walls.y_lenght").get_parameter_value().get<float>();
    wall_h_ = get_parameter("walls.h").get_parameter_value().get<float>();
    wall_w_ = get_parameter("walls.w").get_parameter_value().get<float>();
    wheelradius_ = get_parameter("wheelradius").get_parameter_value().get<float>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<float>();
    encoder_ticks_per_rad_ =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<float>();
    motor_cmd_per_rad_sec_ =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<float>();
    input_noise_ = get_parameter("input_noise").get_parameter_value().get<float>();
    slip_fraction_ = get_parameter("slip_fraction").get_parameter_value().get<float>();

    // Set current robot pose equal to initial pose
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;

    // Create obstacles
    create_obstacles_array();
    // Create walls
    create_walls_array();

    // Update object with params
    turtle_ = turtlelib::DiffDrive{wheelradius_, track_width_};

    // Publishers
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    obstacles_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    walls_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data", 10);
    red_turtle_publisher_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);

    //Subscribers
    red_wheel_cmd_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(
        &Nusim::red_wheel_cmd_callback, this,
        std::placeholders::_1));

    // Reset service
    reset_server_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    // Teleport service
    teleport_server_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&Nusim::timer_callback, this));
  }

private:
  // Variables
  size_t timestep_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  float x_, y_, theta_;     // Theta in radians, x & y in meters.
  float x0_ = 0;
  float y0_ = 0;
  float theta0_ = 0;
  float obstacles_r_;    // Size of obstacles
  float obstacles_h_;
  float wall_h_;   // Size of walls
  float wall_w_;
  float walls_x_;    // Location of walls
  float walls_y_;
  float wheelradius_;
  float track_width_;
  float encoder_ticks_per_rad_;
  float motor_cmd_per_rad_sec_;
  float input_noise_;
  float slip_fraction_;
  std::vector<double> obstacles_x_;    // Location of obstacles
  std::vector<double> obstacles_y_;
  visualization_msgs::msg::MarkerArray obstacles_;
  visualization_msgs::msg::MarkerArray walls_;
  nuturtlebot_msgs::msg::SensorData sensor_data_;
  geometry_msgs::msg::PoseStamped red_pose_stamped_;
  nav_msgs::msg::Path red_path_;
  turtlelib::Wheel new_wheel_pos_;
  turtlelib::Wheel old_wheel_pos_{0.0, 0.0};
  turtlelib::WheelVelocities new_wheel_vel_{0.0, 0.0};
  turtlelib::DiffDrive turtle_;

  // Create objects
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr red_turtle_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr red_wheel_cmd_subscriber_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;

  /// \brief Subscription callback function for wheel_cmd topic
  void red_wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    // Convert wheel cmd ticks to rad/sec
    new_wheel_vel_.left = msg.left_velocity * motor_cmd_per_rad_sec_;
    new_wheel_vel_.right = msg.right_velocity * motor_cmd_per_rad_sec_;
  }

  /// \brief Updates the red turtle's configuration
  void update_red_turtle_config()
  {
    // TODO Rename to change in wheel position
    // OR TODO make this new wheel position then add minus back in FK
    // This will be updated
    new_wheel_pos_.left = new_wheel_vel_.left * 0.005;   // Change in position
    new_wheel_pos_.right = new_wheel_vel_.right * 0.005;   // Change in position
    turtle_.ForwardKinematics(new_wheel_pos_);     // Update robot position
    x_ = turtle_.configuration().x;
    y_ = turtle_.configuration().y;
    theta_ = turtle_.configuration().theta;
    update_sensor_data();
  }

  /// \brief Generates the encoder/sensor_data
  void update_sensor_data()
  {
    // Sensor Data
    new_wheel_pos_.left = old_wheel_pos_.left + new_wheel_vel_.left * 0.005;
    new_wheel_pos_.right = old_wheel_pos_.right + new_wheel_vel_.right * 0.005;
    sensor_data_.stamp = get_clock()->now();
    sensor_data_.left_encoder = new_wheel_pos_.left * encoder_ticks_per_rad_;
    sensor_data_.right_encoder = new_wheel_pos_.right * encoder_ticks_per_rad_;
    old_wheel_pos_.left = new_wheel_pos_.left;
    old_wheel_pos_.right = new_wheel_pos_.right;
  }

  /// \brief Reset the simulation
  void reset_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    timestep_ = 0;
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
  }

  /// \brief Teleport the robot to a specified pose
  void teleport_callback(
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
  }

  /// \brief Broadcast the TF frames of the robot
  void broadcast_red_turtle()
  {
    geometry_msgs::msg::TransformStamped t_;

    t_.header.stamp = get_clock()->now();
    t_.header.frame_id = "nusim/world";
    t_.child_frame_id = "red/base_footprint";
    t_.transform.translation.x = x_;
    t_.transform.translation.y = y_;
    t_.transform.translation.z = 0.0;     // Turtle only exists in 2D

    tf2::Quaternion q_;
    q_.setRPY(0, 0, theta_);     // Rotation around z-axis
    t_.transform.rotation.x = q_.x();
    t_.transform.rotation.y = q_.y();
    t_.transform.rotation.z = q_.z();
    t_.transform.rotation.w = q_.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t_);

    red_turtle_NavPath();
  }

  /// \brief Create the red turtle's nav_msgs/Path
  void red_turtle_NavPath()
  {
    // Update ground truth red turtle path
    red_path_.header.stamp = get_clock()->now();
    red_path_.header.frame_id = "nusim/world";
    // Create new pose stamped
    red_pose_stamped_.header.stamp = get_clock()->now();
    red_pose_stamped_.header.frame_id = "nusim/world";
    red_pose_stamped_.pose.position.x = x_;
    red_pose_stamped_.pose.position.y = y_;
    red_pose_stamped_.pose.position.z = 0.0;
    tf2::Quaternion q_;
    q_.setRPY(0, 0, theta_);     // Rotation around z-axis
    red_pose_stamped_.pose.orientation.x = q_.x();
    red_pose_stamped_.pose.orientation.y = q_.y();
    red_pose_stamped_.pose.orientation.z = q_.z();
    red_pose_stamped_.pose.orientation.w = q_.w();
    // Append pose stamped
    red_path_.poses.push_back(red_pose_stamped_);
  }

  /// \brief Create obstacles as a MarkerArray and publish them to a topic to display them in Rviz
  void create_obstacles_array()
  {
    if (obstacles_x_.size() != obstacles_y_.size()) {
      int err_ = true;
      RCLCPP_ERROR(this->get_logger(), "x and y coordinate lists are not the same lenght!");
      throw err_;
    }

    for (size_t i = 0; i < obstacles_x_.size(); i++) {
      visualization_msgs::msg::Marker obstacle_;
      obstacle_.header.frame_id = "nusim/world";
      obstacle_.header.stamp = get_clock()->now();
      obstacle_.id = i;
      obstacle_.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle_.action = visualization_msgs::msg::Marker::ADD;
      obstacle_.pose.position.x = obstacles_x_[i];
      obstacle_.pose.position.y = obstacles_y_[i];
      obstacle_.pose.position.z = obstacles_h_ / 2.0;
      obstacle_.pose.orientation.x = 0.0;
      obstacle_.pose.orientation.y = 0.0;
      obstacle_.pose.orientation.z = 0.0;
      obstacle_.pose.orientation.w = 1.0;
      obstacle_.scale.x = obstacles_r_ * 2.0;   // Diameter in x
      obstacle_.scale.y = obstacles_r_ * 2.0;   // Diameter in y
      obstacle_.scale.z = obstacles_h_;         // Height
      obstacle_.color.r = 1.0f;
      obstacle_.color.g = 0.0f;
      obstacle_.color.b = 0.0f;
      obstacle_.color.a = 1.0;
      obstacles_.markers.push_back(obstacle_);
    }
  }

  /// \brief Create walls as a MarkerArray and publish them to a topic to display them in Rviz
  void create_walls_array()
  {
    for (int i = 0; i <= 3; i++) {
      visualization_msgs::msg::Marker wall_;
      wall_.header.frame_id = "nusim/world";
      wall_.header.stamp = get_clock()->now();
      wall_.id = i;
      wall_.type = visualization_msgs::msg::Marker::CUBE;
      wall_.action = visualization_msgs::msg::Marker::ADD;

      if (i == 0 || i == 1) {
        wall_.pose.position.x = 0.0;
      } else if (i == 2) {
        wall_.pose.position.x = (walls_x_ + wall_w_) / 2;
      } else if (i == 3) {
        wall_.pose.position.x = -(walls_x_ + wall_w_) / 2;
      }

      if (i == 2 || i == 3) {
        wall_.pose.position.y = 0.0;
      } else if (i == 0) {
        wall_.pose.position.y = (walls_y_ + wall_w_) / 2;
      } else if (i == 1) {
        wall_.pose.position.y = -(walls_y_ + wall_w_) / 2;
      }
      wall_.pose.position.z = wall_h_ / 2.0;

      if (i == 0 || i == 1) {
        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.0;
        wall_.pose.orientation.w = 1.0;
      } else {
        wall_.pose.orientation.x = 0.0;
        wall_.pose.orientation.y = 0.0;
        wall_.pose.orientation.z = 0.7071068;
        wall_.pose.orientation.w = 0.7071068;
      }

      if (i == 0 || i == 1) {
        wall_.scale.x = walls_x_ + 2 * wall_w_;
      } else {
        wall_.scale.x = walls_y_ + 2 * wall_w_;
      }
      wall_.color.r = 1.0f;
      wall_.color.g = 0.0f;
      wall_.color.b = 0.0f;
      wall_.color.a = 1.0;
      wall_.scale.y = wall_w_;
      wall_.scale.z = wall_h_;
      walls_.markers.push_back(wall_);
    }
  }

  /// \brief Main simulation timer loop
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;
    timestep_publisher_->publish(message);
    obstacles_publisher_->publish(obstacles_);
    walls_publisher_->publish(walls_);
    update_red_turtle_config();
    sensor_data_publisher_->publish(sensor_data_);
    broadcast_red_turtle();
    red_turtle_publisher_->publish(red_path_);
  }
};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Nusim>());
  } catch (int err_) {
    RCLCPP_ERROR(
      std::make_shared<Nusim>()->get_logger(), "x and y coordinate lists are not the same lenght!");
  }
  rclcpp::shutdown();
  return 0;
}
