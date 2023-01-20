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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Nusim : public rclcpp::Node
{
  public:
    Nusim(): Node("Nusim"), timestep_(0)
    {
      // Parameters
      declare_parameter("rate", 200); // Hz for timer_callback
      // Initial pose - Set default as 0
      declare_parameter("x0", 0.0);
      declare_parameter("y0", 0.0);
      declare_parameter("theta0", 0.0);
      declare_parameter("obstacles.x", std::vector<double>{});
      declare_parameter("obstacles.y", std::vector<double>{});
      declare_parameter("obstacles.r", 0.0);
      declare_parameter("obstacles.h", 0.0);
      // Get params - Read params from yaml file that is passed in the launch file
      int rate = get_parameter("rate").get_parameter_value().get<int>();
      x0 = get_parameter("x0").get_parameter_value().get<float>();
      y0 = get_parameter("y0").get_parameter_value().get<float>();
      theta0 = get_parameter("theta0").get_parameter_value().get<float>();
      obstacles_x = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
      obstacles_y = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
      obstacles_r = get_parameter("obstacles.r").get_parameter_value().get<float>();
      obstacles_h = get_parameter("obstacles.h").get_parameter_value().get<float>();
      // Current robot pose
      x = x0;
      y = y0;
      theta = theta0;

      // Create obstacles
      obstacles_ = create_obstacles_array(obstacles_x, obstacles_y, obstacles_r, obstacles_h);

      // Publisher
      timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

      // Reset service
      reset_server_ = create_service<std_srvs::srv::Empty>("~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
      // Teleport service
      teleport_server_ = create_service<nusim::srv::Teleport>("~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

      // Initialize the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Timer
      timer_ = create_wall_timer(std::chrono::milliseconds(1000/rate),
      std::bind(&Nusim::timer_callback, this));
    }

  private:
    // Variables
    size_t timestep_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    float x, y, theta; // Theta in radians, x & y in meters.
    float x0 = 0;
    float y0 = 0;
    float theta0 = 0;
    float obstacles_r;
    float obstacles_h;
    std::vector<double> obstacles_x;
    std::vector<double> obstacles_y;
    visualization_msgs::msg::MarkerArray obstacles_;

    // Create objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_server_;

    // Reset service callback
    void reset_callback(std_srvs::srv::Empty::Request::SharedPtr ,std_srvs::srv::Empty::Response::SharedPtr)
    {
      timestep_ = 0; // Reset timestep
      x = x0;
      y = y0;
      theta = theta0;
    }

    // teleport service callback
    void teleport_callback(nusim::srv::Teleport::Request::SharedPtr request, nusim::srv::Teleport::Response::SharedPtr)
    {
      x = request->x;
      y = request->y;
      theta = request->theta;
    }

    void broadcast_red_turtle()
    {
      geometry_msgs::msg::TransformStamped t;

      // Read message content and assign it to
      // corresponding tf variables
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "nusim/world";
      t.child_frame_id = "red/base_footprint";

      // Turtle only exists in 2D, thus we get x and y translation
      // coordinates from the message and set the z coordinate to 0
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.translation.z = 0.0;

      // For the same reason, turtle can only rotate around one axis
      // and this why we set rotation in x and y to 0 and obtain
      // rotation in z axis from the message
      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Send the transformation
      tf_broadcaster_->sendTransform(t);
    }

     visualization_msgs::msg::MarkerArray create_obstacles_array(std::vector<double> obstacles_x,
                                                                 std::vector<double> obstacles_y,
                                                                 float obstacles_r,
                                                                 float obstacles_h)
    {

      if (obstacles_x.size()!=obstacles_y.size())
      {
        int err = true;
        RCLCPP_ERROR(this->get_logger(), "x and y coordinate lists are not the same lenght!");
        throw err;
      }

      // Create array for obstacles
      visualization_msgs::msg::MarkerArray obstacles_array;

      for (int i = 0; i<(int)obstacles_x.size(); i++)
      {
        visualization_msgs::msg::Marker obstacle;
        obstacle.header.frame_id = "nusim/world";
        obstacle.header.stamp = this->get_clock()->now();
        obstacle.id = i;
        obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
        obstacle.action = visualization_msgs::msg::Marker::ADD;
        obstacle.pose.position.x = obstacles_x[i];
        obstacle.pose.position.y = obstacles_y[i];
        obstacle.pose.position.z = obstacles_h/2.0;
        obstacle.pose.orientation.x = 0.0;
        obstacle.pose.orientation.y = 0.0;
        obstacle.pose.orientation.z = 0.0;
        obstacle.pose.orientation.w = 1.0;
        obstacle.scale.x = obstacles_r*2.0;
        obstacle.scale.y = obstacles_r*2.0;
        obstacle.scale.z = obstacles_h;
        obstacle.color.r = 0.3058f;
        obstacle.color.g = 0.1647f;
        obstacle.color.b = 0.5176f;
        obstacle.color.a = 1.0;
        obstacles_array.markers.push_back(obstacle);
      }

      return obstacles_array;
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::UInt64();
      message.data = timestep_++;
      // LEAVE - RCLCPP_INFO_STREAM(get_logger(), "Publishing:  " << message.data);
      timestep_publisher_->publish(message);
      obstacles_publisher_->publish(obstacles_);
      broadcast_red_turtle();
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<Nusim>());
  }
  catch(int err)
  {
    RCLCPP_ERROR(std::make_shared<Nusim>()->get_logger(), "x and y coordinate lists are not the same lenght!");
  }
  rclcpp::shutdown();
  return 0;
}