/// \file
/// \brief The nusim package is a simulation and visualization tool for the turtlebot3 robots.
///        It uses rviz2 for visualization and provides a simulated environment. The package
///        will be able to create stationary walls and obstacles and track the position of a
///        robot.
///
/// PARAMETERS:
///     \param rate (int): Timer callback frequency [Hz]
///     \param x0_ (float): Initial x coordinate of the robot [m]
///     \param y0_ (float): Initial y coordinate of the robot [m]
///     \param theta0_ (float): Initial theta angle of the robot [radians]
///     \param obstacles.x_ (std::vector<double>): Vector of x coordinates for each obstacle [m]
///     \param obstacles.y_ (std::vector<double>): Vector of y coordinates for each obstacle [m]
///     \param obstacles.r_ (float): Radius of cylindrical obstacles [m]
///     \param obstacles.h_ (float): Height of cylindrical obstacles [m]
///
/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): Current simulation timestep
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): Marker obstacles that are
///                                                                displayed in Rviz
///
/// SUBSCRIBES:
///     None
///
/// SERVERS:
///     \param ~/reset (std_srvs::srv::Empty): Resets simulation to initial state
///     \param ~/teleport (nusim::srv::Teleport): Teleport robot to a specific pose
///
/// CLIENTS:
///     None

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

using namespace std::chrono_literals;

/// \brief This class publishes the current timestep of the simulation and obstacles that
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
///  \param obstacles.x_ (std::vector<double>): Vector of x coordinates for each obstacle [m]
///  \param obstacles.y_ (std::vector<double>): Vector of y coordinates for each obstacle [m]
///  \param obstacles.r_ (float): Radius of cylindrical obstacles [m]
///  \param obstacles.h_ (float): Height of cylindrical obstacles [m]

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
      rate_des.description = "Timer callback frequency [Hz]";
      x0_des.description = "Initial x coordinate of the robot [m]";
      y0_des.description = "Initial y coordinate of the robot [m]";
      theta0_des.description = "Initial theta angle of the robot [radians]";
      obstacles_x_des.description = "Vector of x coordinates for each obstacle [m]";
      obstacles_y_des.description = "Vector of y coordinates for each obstacle [m]";
      obstacles_r_des.description = "Radius of cylindrical obstacles [m]";
      obstacles_h_des.description = "Height of cylindrical obstacles [m]";

      // Declare default parameters values
      declare_parameter("rate", 200, rate_des);   // Hz for timer_callback
      declare_parameter("x0", 0.0, x0_des);
      declare_parameter("y0", 0.0, y0_des);
      declare_parameter("theta0", 0.0, theta0_des);
      declare_parameter("obstacles.x", std::vector<double>{}, obstacles_x_des);
      declare_parameter("obstacles.y", std::vector<double>{}, obstacles_y_des);
      declare_parameter("obstacles.r", 0.0, obstacles_r_des);
      declare_parameter("obstacles.h", 0.0, obstacles_h_des);
      // Get params - Read params from yaml file that is passed in the launch file
      int rate = get_parameter("rate").get_parameter_value().get<int>();
      x0_ = get_parameter("x0").get_parameter_value().get<float>();
      y0_ = get_parameter("y0").get_parameter_value().get<float>();
      theta0_ = get_parameter("theta0").get_parameter_value().get<float>();
      obstacles_x_ = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
      obstacles_y_ = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
      obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<float>();
      obstacles_h_ = get_parameter("obstacles.h").get_parameter_value().get<float>();

      // Set current robot pose equal to initial pose
      x_ = x0_;
      y_ = y0_;
      theta_ = theta0_;

      // Create obstacles
      create_obstacles_array();

      // Publisher
      timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      obstacles_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

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
    float x_, y_, theta_;   // Theta in radians, x & y in meters.
    float x0_ = 0;
    float y0_ = 0;
    float theta0_ = 0;
    float obstacles_r_;
    float obstacles_h_;
    std::vector<double> obstacles_x_;
    std::vector<double> obstacles_y_;
    visualization_msgs::msg::MarkerArray obstacles_;

    // Create objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;

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

      t_.header.stamp = this->get_clock()->now();
      t_.header.frame_id = "nusim/world";
      t_.child_frame_id = "red/base_footprint";
      t_.transform.translation.x = x_;
      t_.transform.translation.y = y_;
      t_.transform.translation.z = 0.0;   // Turtle only exists in 2D

      tf2::Quaternion q_;
      q_.setRPY(0, 0, theta_);   // Rotation around z-axis
      t_.transform.rotation.x = q_.x();
      t_.transform.rotation.y = q_.y();
      t_.transform.rotation.z = q_.z();
      t_.transform.rotation.w = q_.w();

      // Send the transformation
      tf_broadcaster_->sendTransform(t_);
    }

    /// \brief Create obstacles as a MarkerArray and publish them to a topic to display them in Rviz
    void create_obstacles_array()
    {
      if (obstacles_x_.size() != obstacles_y_.size()) {
        int err_ = true;
        RCLCPP_ERROR(this->get_logger(), "x and y coordinate lists are not the same lenght!");
        throw err_;
      }

      for (int i = 0; i < (int)obstacles_x_.size(); i++) {
        visualization_msgs::msg::Marker obstacle_;
        obstacle_.header.frame_id = "nusim/world";
        obstacle_.header.stamp = this->get_clock()->now();
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
        obstacle_.scale.x = obstacles_r_ * 2.0; // Diameter in x
        obstacle_.scale.y = obstacles_r_ * 2.0; // Diameter in y
        obstacle_.scale.z = obstacles_h_;       // Height
        obstacle_.color.r = 1.0f;
        obstacle_.color.g = 0.0f;
        obstacle_.color.b = 0.0f;
        obstacle_.color.a = 1.0;
        obstacles_.markers.push_back(obstacle_);
      }
    }

    /// \brief Main simulation timer loop
    void timer_callback()
    {
      auto message = std_msgs::msg::UInt64();
      message.data = timestep_++;
      timestep_publisher_->publish(message);
      obstacles_publisher_->publish(obstacles_);
      broadcast_red_turtle();
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
