#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


class odometry : public rclcpp::Node
{
  public:
    odometry()
    : Node("odometry")
    {

    }

  private:

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odometry>());
    rclcpp::shutdown();
    return 0;
}