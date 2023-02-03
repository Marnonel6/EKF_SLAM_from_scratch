#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


class circle : public rclcpp::Node
{
  public:
    circle()
    : Node("circle")
    {

    }

  private:

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<circle>());
    rclcpp::shutdown();
    return 0;
}