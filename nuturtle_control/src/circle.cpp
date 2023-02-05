#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class circle : public rclcpp::Node
{
  public:
    circle()
    : Node("circle")
    {
        // Publishers
        cmd_vel_subscriber_ = create_publisher<geometry_msgs::msg::Twist>(
                               "cmd_vel", 10);

        // Timer
        timer_ = create_wall_timer(10ms, std::bind(&circle::timer_callback, this));
    }

  private:
    // Variables
    geometry_msgs::msg::Twist body_twist_;

    // Create objects
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    /// \brief Main timer loop
    void timer_callback()
    {
        cmd_vel_subscriber_->publish(body_twist_);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<circle>());
    rclcpp::shutdown();
    return 0;
}