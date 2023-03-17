/// \file
/// \brief
///
/// PARAMETERS:
///     \param obstacles.r (double): Radius of cylindrical obstacles [m]
///     \param obstacles.h (double): Height of cylindrical obstacles [m]
///
/// PUBLISHES:
///     \param ~/clusters (visualization_msgs::msg::MarkerArray): Clusters MarkerArray as seen by
///                                                               Clustering algorithm
///     \param ~/circle_fit (visualization_msgs::msg::MarkerArray): Fitted circles MarkerArray as
///                                                                 seen by circle fitting algorithm
///
/// SUBSCRIBES:
///     \param /nusim/fake_lidar_scan (sensor_msgs::msg::LaserScan): Subscribes to the fake lidar
///                                                                  points
///
/// SERVERS:
///     None
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     None

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
#include "turtlelib/circle_fitting.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtle_control/srv/initial_config.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

/// \brief
///
///  \param obstacles_r_ (double): Radius of cylindrical obstacles [m]
///  \param obstacles_h_ (double): Height of cylindrical obstacles [m]

class landmarks : public rclcpp::Node
{
public:
  landmarks()
  : Node("landmarks")
  {
    // Parameter descirption
    auto obstacles_r_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_h_des = rcl_interfaces::msg::ParameterDescriptor{};
    obstacles_r_des.description = "Radius of cylindrical obstacles [m]";
    obstacles_h_des.description = "Height of cylindrical obstacles [m]";
    // Declare default parameters values
    declare_parameter("obstacles.r", 0.0, obstacles_r_des);
    declare_parameter("obstacles.h", 0.0, obstacles_h_des);
    // Get params - Read params from yaml file that is passed in the launch file
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    obstacles_h_ = get_parameter("obstacles.h").get_parameter_value().get<double>();

    // Subscribers
    lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/nusim/fake_lidar_scan", 10, std::bind(
        &landmarks::lidar_sensor_callback,
        this, std::placeholders::_1));

    // Publishers
    cluster_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/clusters", 10);
    circle_fit_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/circle_fit", 10);

  }

private:
  // Variables
  bool Flag_cluster = true;
  double threshold_dist_ = 0.1; // Threshold for clustering lidar data
  double obstacles_r_;    // Size of obstacles
  double obstacles_h_;

  // Create objects
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr circle_fit_publisher_;


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
                    temp_cluster.push_back(point_next_i);
                    // Check next point in lidar scan
                    count++;
                }
                else if (count>=3) // We have atleast 4 points in the cluster
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

    circle_fit(clusters); // Fit circle to clusters
    create_clusters_array(clusters); // Create and publish clusters
  }

  /// \brief Run the circle fitting algorithm to get the circle radius and location (x,y)
  void circle_fit(std::vector<std::vector<turtlelib::Vector2D>> clusters)
  {
    std::vector<turtlelib::Circle> circle_list{};

    // Iterate through clusters and pass to circle fitting function
    for (size_t i = 0; i < clusters.size(); i++)
    {
        turtlelib::Circle circle_params = turtlelib::circle_fitting(clusters.at(i));
        if (circle_params.R < 0.1 && circle_params.R > 0.01) // Filter circle for radii smaller than 0.1 and greater than 0.01
        {
            circle_list.push_back(circle_params);
        }
    }

    create_circles_array(circle_list); // Publish fitted circles as a MarkerArray

  }

  /// \brief Create circle fitted MarkerArray as seen by Circle fitting algorithm and publish them to a topic to display them in Rviz
  void create_circles_array(std::vector<turtlelib::Circle> circle_list)
  {
    visualization_msgs::msg::MarkerArray circles_;

    for (size_t i = 0; i < circle_list.size(); i++)
    {
        visualization_msgs::msg::Marker circle_;
        circle_.header.frame_id = "green/base_footprint";
        circle_.header.stamp = get_clock()->now();
        circle_.id = i;
        circle_.type = visualization_msgs::msg::Marker::CYLINDER;
        circle_.action = visualization_msgs::msg::Marker::ADD;
        circle_.pose.position.x = circle_list.at(i).x;
        circle_.pose.position.y = circle_list.at(i).y;
        circle_.pose.position.z = obstacles_h_/2.0;
        circle_.pose.orientation.x = 0.0;
        circle_.pose.orientation.y = 0.0;
        circle_.pose.orientation.z = 0.0;
        circle_.pose.orientation.w = 1.0;
        circle_.scale.x = circle_list.at(i).R * 2.0; //obstacles_r_ * 2.0;       // Diameter in x
        circle_.scale.y = circle_list.at(i).R * 2.0; //obstacles_r_ * 2.0;       // Diameter in y
        circle_.scale.z = obstacles_h_;             // Height
        circle_.color.r = 0.0f;
        circle_.color.g = 0.0f;
        circle_.color.b = 1.0f;
        circle_.color.a = 1.0;
        circles_.markers.push_back(circle_);
    }
    circle_fit_publisher_->publish(circles_);
  }

  /// \brief Create clusters MarkerArray as seen by Clustering algorithm and publish them to a topic to display them in Rviz
  void create_clusters_array(std::vector<std::vector<turtlelib::Vector2D>> clusters)
  {
    visualization_msgs::msg::MarkerArray clusters_;

    for (size_t i = 0; i < clusters.size(); i++)
    {
        auto x_avg = 0.0;
        auto y_avg = 0.0;
        auto num_elements = 0.0;
        for (size_t j = 0; j < clusters.at(i).size(); j++)
        {
            x_avg += clusters.at(i).at(j).x;
            y_avg += clusters.at(i).at(j).y;
            num_elements += 1.0;
        }

        visualization_msgs::msg::Marker cluster_;
        cluster_.header.frame_id = "green/base_footprint";
        cluster_.header.stamp = get_clock()->now();
        cluster_.id = i;
        cluster_.type = visualization_msgs::msg::Marker::CYLINDER;
        cluster_.action = visualization_msgs::msg::Marker::ADD;
        cluster_.pose.position.x = x_avg/num_elements;
        cluster_.pose.position.y = y_avg/num_elements;
        cluster_.pose.position.z = obstacles_h_/2.0;
        cluster_.pose.orientation.x = 0.0;
        cluster_.pose.orientation.y = 0.0;
        cluster_.pose.orientation.z = 0.0;
        cluster_.pose.orientation.w = 1.0;
        cluster_.scale.x = obstacles_r_ * 2.0;       // Diameter in x
        cluster_.scale.y = obstacles_r_ * 2.0;       // Diameter in y
        cluster_.scale.z = obstacles_h_;             // Height
        cluster_.color.r = 1.0f;
        cluster_.color.g = 0.0f;
        cluster_.color.b = 1.0f;
        cluster_.color.a = 1.0;
        clusters_.markers.push_back(cluster_);
    }
    cluster_publisher_->publish(clusters_);
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
