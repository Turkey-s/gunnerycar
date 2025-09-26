#ifndef FORM_TEAM_STATE_HPP
#define FORM_TEAM_STATE_HPP

#include "state.hpp"
#include "autofleet_node.hpp"
namespace autofleet
{
class FormTeamState : public State
{
private:
    
public:
    FormTeamState(
        const std::string& xml_tag_name, const BT::NodeConfiguration& conf) 
        : State(xml_tag_name, conf){}
    virtual ~FormTeamState() = default;

    virtual void ComputeHeadVel() override; // 计算下一次控制时头车的速度
    virtual void ComputeFollowPose() override; // 计算下一次控制时跟随车全部路径规划的点(x,y,yaw)
    virtual void ComputeFollowVel() override; // 计算下一次控制时到达目标点跟随车的车速

    static BT::PortsList providedPorts()
    {
        return BT::PortsList{};
    }

    virtual BT::NodeStatus tick() override;
};

    
} // namespace autofleet

#endif // STATE_HPP