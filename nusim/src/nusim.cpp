#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
// #include "nusim/srv/Empty.srv"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Nusim : public rclcpp::Node
{
  public:
    Nusim(): Node("Nusim"), timestep_(0)
    {
      // Parameters
      declare_parameter("rate", 200);
      int rate = get_parameter("rate").get_parameter_value().get<int>();

      // Publisher
      publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

      // Reset service
      server_ = create_service<std_srvs::srv::Empty>("~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

      // Timer
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

    size_t timestep_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;


    // Reset service
    void reset_callback(std_srvs::srv::Empty::Request::SharedPtr ,std_srvs::srv::Empty::Response::SharedPtr)
    {
        timestep_ = 0; // Reset timesetp
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}