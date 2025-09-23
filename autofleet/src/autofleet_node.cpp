#include <chrono>
#include "autofleet/autofleet_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autofleet/breakTeamState.hpp"
#include "autofleet/formTeamState.hpp"
#include "autofleet/moveState.hpp"
#include "iostream"

namespace autofleet
{
AutofleetMgrNode::AutofleetMgrNode() : Node("autofleet_node")
{
  RCLCPP_INFO(get_logger(), "Creating autofleet node");
  declare_parameter("bt_xml_file", "bt_autofleet.xml");
  declare_parameter("plugins", std::vector<std::string>{"FormTeamState", "MoveState", "BreakTeamState"});
  sub_target_pose_= this->create_subscription<geometry_msgs::msg::Pose>(
    "target_pose", 10, std::bind(&AutofleetMgrNode::TargetPoseCallback, this, std::placeholders::_1)
  );
}

void AutofleetMgrNode::TargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  CreateTree();
}

void AutofleetMgrNode::CreateTree()
{
  std::vector<std::string> plugin_libraries = get_parameter("plugins").as_string_array();
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }

  auto bt_xml_file = this->get_parameter("bt_xml_file").as_string();
  auto package_share_dir = ament_index_cpp::get_package_share_directory("autofleet");
  bt_xml_file = package_share_dir + "/config/" + bt_xml_file;

  blackboard_ = BT::Blackboard::create();
  blackboard_->set<rclcpp::Node::SharedPtr>("node", shared_from_this());
  
  tree_ = factory_.createTreeFromFile(bt_xml_file, blackboard_);

  tree_.tickRootWhileRunning(std::chrono::milliseconds(30));
}

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autofleet::AutofleetMgrNode>();
  node->CreateTree();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
