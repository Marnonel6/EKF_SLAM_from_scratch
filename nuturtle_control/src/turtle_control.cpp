#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class turtle_control : public rclcpp::Node
{
  public:
    turtle_control()
    : Node("turtle_control")
    {
        // Publisher
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = create_wall_timer(500ms, std::bind(&turtle_control::timer_callback, this));
    }

  private:
    // Variables

    // Create objects
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    /// \brief Main simulation timer loop
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 1;
        RCLCPP_INFO(get_logger(), "Publishing: '%f'", message.linear.x);
        cmd_vel_publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtle_control>());
    rclcpp::shutdown();
    return 0;
}