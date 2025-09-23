#ifndef STATE_H
#define STATE_H

#include "autofleet_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace autofleet
{
class State : public BT::ActionNodeBase
{
public:
    State() = delete;

    State(
        const std::string& xml_tag_name, const BT::NodeConfiguration& conf) 
    : BT::ActionNodeBase(xml_tag_name, conf) 
    {
        xml_tag_name_ = xml_tag_name;
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        RCLCPP_INFO(node_->get_logger(), "%s State node created", xml_tag_name.c_str());
    };
    virtual ~State() = default;

    virtual BT::NodeStatus tick() override
    {
        RCLCPP_INFO(node_->get_logger(), "tick %s", xml_tag_name_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    void halt() override
    {
    }

    virtual void ComputeHeadVel() = 0; // 计算下一次控制时头车的速度
    virtual void ComputeFollowPose() = 0; // 计算下一次控制时跟随车全部路径规划的点(x,y,yaw)
    virtual void ComputeFollowVel() = 0; // 计算下一次控制时到达目标点跟随车的车速
    
    static BT::PortsList providedPorts()
    {
    }

    void GetHeadPath();
private:
    AutofleetMgrNode::SharedPtr node_;
    std::string xml_tag_name_;
};
} // namespace autofleet

#endif // STATE_H