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
        auto node_ = config().blackboard->get<AutofleetMgrNode::SharedPtr>("node");
        if(node_ != nullptr)
        {
            mgr_node_ = std::dynamic_pointer_cast<AutofleetMgrNode>(node_);
        }
        else
        {
            RCLCPP_ERROR(mgr_node_->get_logger(), "autofleet node point is nullptr");
        }
        RCLCPP_INFO(mgr_node_->get_logger(), "%s State node created", xml_tag_name.c_str());
    };
    virtual ~State() = default;

    virtual BT::NodeStatus tick() override
    {
        RCLCPP_INFO(mgr_node_->get_logger(), "tick %s", xml_tag_name_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    void halt() override
    {
    }

    virtual void ComputeHeadVel() = 0; // 计算下一次控制时头车的速度
    virtual void ComputeFollowPose();
    virtual void ComputeFollowVel() = 0; // 计算下一次控制时到达目标点跟随车的车速
    
    static BT::PortsList providedPorts()
    {
        return BT::PortsList{};
    }
private:
    std::shared_ptr<AutofleetMgrNode> mgr_node_;
    std::string xml_tag_name_;
};

/* 一字型，车间2.0 */
//依据当前头车的目标点，计算跟随车的目标点，跟随车的目标点为头车path上的某一个点的相对位置，如有左右偏移，那就是以那一点的左右偏移量
void State::ComputeFollowPose()
{
    if(mgr_node_ == nullptr)
    {
        RCLCPP_INFO(mgr_node_->get_logger(), "autofleet node is nullptr");
        return;
    }

    auto head_path = mgr_node_->GetHeadPath();
}

} // namespace autofleet

#endif // STATE_H