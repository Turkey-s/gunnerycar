#include "rclcpp/rclcpp.hpp"
#include "autofleet/autofleet_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autofleet::AutofleetMgrNode>();
  node->Run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  std::cout << node.use_count() << std::endl;
  return 0;
}
