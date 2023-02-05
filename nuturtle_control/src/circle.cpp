#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/initial_config.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class circle : public rclcpp::Node
{
  public:
    circle()
    : Node("circle")
    {
        // Parameter descirption
        auto frequency_des = rcl_interfaces::msg::ParameterDescriptor{};
        frequency_des.description = "Timer callback frequency [Hz]";
        // Declare default parameters values
        declare_parameter("frequency", 100, frequency_des);   // Hz for timer_callback
        // Get params - Read params from yaml file that is passed in the launch file
        int frequency = get_parameter("frequency").get_parameter_value().get<int>();

        // Publishers
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
                               "cmd_vel", 10);

        // Control server
        control_server_ = create_service<nuturtle_control::srv::Control>("control",
        std::bind(&circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));
        // Reverse server
        reverse_server_ = create_service<std_srvs::srv::Empty>("reverse",
        std::bind(&circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));
        // Stop server
        stop_server_ = create_service<std_srvs::srv::Empty>("stop",
        std::bind(&circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Timer
        timer_ = create_wall_timer(std::chrono::milliseconds(1000 / frequency),
                                   std::bind(&circle::timer_callback, this));
    }

  private:
    // Variables
    bool Flag_stop_ = false;
    geometry_msgs::msg::Twist body_twist_;

    // Create objects
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_server_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_server_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
    rclcpp::TimerBase::SharedPtr timer_;

    /// \brief control service callback
    void control_callback(nuturtle_control::srv::Control::Request::SharedPtr request,
                          nuturtle_control::srv::Control::Response::SharedPtr)
    {
        body_twist_.linear.x = request->radius*request->velocity;
        body_twist_.angular.z = request->velocity;
        Flag_stop_ = false;
    }

    /// \brief reverse service callback
    void reverse_callback(std_srvs::srv::Empty::Request::SharedPtr,
                          std_srvs::srv::Empty::Response::SharedPtr)
    {
        body_twist_.linear.x = -body_twist_.linear.x;
        body_twist_.angular.z = -body_twist_.angular.z;
        Flag_stop_ = false;
    }

    /// \brief stop service callback
    void stop_callback(std_srvs::srv::Empty::Request::SharedPtr,
                          std_srvs::srv::Empty::Response::SharedPtr)
    {
        body_twist_.linear.x = 0.0;
        body_twist_.angular.z = 0.0;
        cmd_vel_publisher_->publish(body_twist_);
        Flag_stop_ = true;
    }

    /// \brief Main timer loop
    void timer_callback()
    {
        if (Flag_stop_ == false)
        {
            cmd_vel_publisher_->publish(body_twist_);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<circle>());
    rclcpp::shutdown();
    return 0;
}