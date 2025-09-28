#ifndef FORM_TEAM_STATE_HPP
#define FORM_TEAM_STATE_HPP

#include "state.hpp"
#include "autofleet_node.hpp"
namespace autofleet
{
class FormTeamState : public State
{
private:
    enum stage{
        STAGE_NONE,
        STAGE_HEAD_MOVE,
        STAGE_FOLLOW_MOVE,
    }; // 车队状态
public:
    FormTeamState(
        const std::string& xml_tag_name, const BT::NodeConfiguration& conf) 
        : State(xml_tag_name, conf){}
    virtual ~FormTeamState() = default;

    bool FollowCanMove(PoseStamp& follow_pose, std::string robot_name); // 判断某个车是否可以开始移动
    bool FollowMovedEnd(PoseStamp& follow_pose, std::string robot_name); // 判断某个车是否到达目标点

    static BT::PortsList providedPorts()
    {
        return BT::PortsList{};
    }

    virtual BT::NodeStatus tick() override;

private:
    stage m_stage{STAGE_NONE}; // 车队状态
    int m_follow_moving_index = 0; // 当前正在移动的跟随车索引(默认值是0，0是头车)
};

    
} // namespace autofleet

#endif // STATE_HPP