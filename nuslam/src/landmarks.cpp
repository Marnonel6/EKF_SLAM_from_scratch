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
#include "sensor_msgs/msg/laser_scan.hpp"

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

    // Subscribers
    lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/nusim/fake_lidar_scan", 10, std::bind(
        &landmarks::lidar_sensor_callback,
        this, std::placeholders::_1));



  }

private:
  // Variables
  bool Flag_cluster = true;
  double threshold_dist_ = 0.1; // Threshold for clustering lidar data

  // Create objects
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;


  /// \brief Lidar sensor topic callback
  void lidar_sensor_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<std::vector<turtlelib::Vector2D>> clusters; // Vector of clusters
    bool Flag_cluster_wrap_around = false; // Flag that notifies if wrap around of a cluster has occured

    for (size_t i = 0; i < msg.ranges.size(); i++) //  Loop through one cycle of lidar points
    {
        if (msg.ranges.at(i) > 0.01) // Check if the lidar point hit an object
        {
            std::vector<turtlelib::Vector2D> temp_cluster{}; // Create a possible cluster
            Flag_cluster = true;
            size_t count = 0;

            while(Flag_cluster)
            {
                turtlelib::Vector2D point_i = PolarToCartesian(msg.ranges.at((i + count)%360), turtlelib::normalize_angle(((i+count)%360)*turtlelib::PI/180.0));
                if (count == 0)
                {
                    temp_cluster.push_back(point_i);
                }
                turtlelib::Vector2D point_next_i = PolarToCartesian(msg.ranges.at((i + 1 + count)%360), turtlelib::normalize_angle(((i+1+count)%360)*turtlelib::PI/180.0));
                double distance_to_next = euclidean_distance(point_i.x, point_i.y, point_next_i.x, point_next_i.y);

                if (distance_to_next < threshold_dist_)
                {
                    // Save to vector
                    RCLCPP_ERROR_STREAM(get_logger(), (i+count)%360 << " " << (i+1+count)%360 << " ");
                    temp_cluster.push_back(point_next_i);
                    // Check next point in lidar scan
                    count++;
                }
                else if (count>=2) // We have atleast 3 points in the cluster
                {
                    // End of cluster
                    Flag_cluster = false;
                    i = i + count - 1;

                    // Save cluster to list of clusters
                    clusters.push_back(temp_cluster);
                }
                else
                {
                    Flag_cluster = false;
                }

                // Check if wrap around of clusters has occured > 360 Degree
                if (i+1+count >= 360)
                {
                    Flag_cluster_wrap_around = true;
                }
            }
        }
    }

    // If wrap around has occured check if the last cluster contains the first cluster
    if (Flag_cluster_wrap_around == true)
    {
        if (clusters.at(0).back().x == clusters.back().back().x && clusters.at(0).back().y == clusters.back().back().y) //  Only delete if if there is an encapsuled cluster
        {
            clusters.at(0) = clusters.back(); // Save last cluster as first
            clusters.pop_back(); // Delete last cluster
        }

        // TODO - Better than checking exact values above???????
        // if (euclidean_distance(clusters.at(0).back().x, clusters.at(0).back().y, clusters.back().back().x, clusters.back().back().y) < threshold_dist_)
        // {
        //     clusters.at(0) = clusters.back(); // Save last cluster as first
        //     clusters.pop_back(); // Delete last cluster
        // }
    }


    // for (size_t i = 0; i<clusters.size(); i++)
    // {
    //     for (size_t j = 0; j<clusters.at(i).size(); j++)
    //     {
    //         RCLCPP_ERROR_STREAM(get_logger(), "\n" << clusters.at(i).at(j));
    //     }
    // }

  }

  /// \brief Calculate the euclidean distance
  /// \param x1 point 1 x-coordinate (double)
  /// \param y1 point 1 y-coordinate (double)
  /// \param x2 point 2 x-coordinate (double)
  /// \param y2 point 2 y-coordinate (double)
  /// \return euclidean distance (double)
  double euclidean_distance(double x1, double y1, double x2, double y2)
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }

  /// \brief Calculate the x,y coordinates from range and bearing
  /// \param range distance to point [m] (double)
  /// \param theta angle to point [radians] (double)
  /// \return 2D vector x and y (turtlelib::Vector2D)
  turtlelib::Vector2D PolarToCartesian(double range, double theta)
  {
    return {range*cos(theta), range*sin(theta)};
  }

};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<landmarks>());
  rclcpp::shutdown();
  return 0;
}
