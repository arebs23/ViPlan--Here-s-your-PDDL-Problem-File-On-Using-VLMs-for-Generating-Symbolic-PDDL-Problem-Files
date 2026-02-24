#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Viplan : public rclcpp::Node
{
  public:
    Viplan()
    : Node("viplan")
    {
      RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "viplan");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Viplan>());
  rclcpp::shutdown();
  return 0;
}