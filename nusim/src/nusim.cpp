#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Nusim : public rclcpp::Node
{
  public:
    Nusim()
    : Node("Nusim"), timestep_(0)
    {
      declare_parameter("rate", 200);
      int rate = get_parameter("rate").get_parameter_value().get<int>();
      publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      timer_ = create_wall_timer(std::chrono::milliseconds(1000/rate),
      std::bind(&Nusim::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::UInt64();
      message.data = timestep_++;
      RCLCPP_INFO_STREAM(get_logger(), "Publishing:  " << message.data);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    size_t timestep_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}