/// \file
/// \brief The slam node subscribes to joint_states and publishes to the green/odom navigation
///        topic. The node handles the SLAM calculations of the green robot.
///
/// PARAMETERS:
///     \param body_id (std::string): The name of the body frame of the robot
///
/// PUBLISHES:
///     \param /odom (nav_msgs::msg::Odometry): Odometry publisher
///
/// SUBSCRIBES:
///     \param /joint_states (sensor_msgs::msg::JointState): Subscribes joint states for green robot
///
/// SERVERS:
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts green turtle position relative to odom

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

/// \brief The class subscribes to
///
///  \param body_id_ (std::string): The name of the body frame of the robot

class landmarks : public rclcpp::Node
{
public:
  landmarks()
  : Node("landmarks")
  {


  }

private:
  // Variables

};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<landmarks>());
  rclcpp::shutdown();
  return 0;
}
