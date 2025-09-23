#ifndef AUTOFLEET_NODE
#define AUTOFLEET_NODE

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace autofleet
{
class AutofleetMgrNode : public rclcpp::Node
{
public:
    AutofleetMgrNode();
    void CreateTree(); // 创建行为树

private:
    void TargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

private:
    // 行为树相关
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;

    std::string bt_xml_filename_;
    
    // Node相关
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_target_pose_;
};

};

#endif