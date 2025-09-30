#include "rclcpp/rclcpp.hpp"
#include "autofleet/autofleet_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<autofleet::AutofleetMgrNode>();
  exec.add_node(node);
  node->Run();
  exec.spin();
  rclcpp::shutdown();
  std::cout << node.use_count() << std::endl;
  return 0;
}
