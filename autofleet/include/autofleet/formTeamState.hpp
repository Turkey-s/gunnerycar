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

    virtual void SetFollowPose(VecPoseStampPtr follow_poses_ptr) override; // 设置跟随车的目标位置

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<VecPoseStampPtr>("output_last_follow_poses_ptr") };
    }

    virtual BT::NodeStatus tick() override;

private:
    bool follow_can_move(PoseStamp& follow_pose, std::string robot_name); // 判断某个车是否可以开始移动

private:
    stage m_stage_{STAGE_NONE}; // 车队状态
    int m_follow_moving_index_ = 0; // 当前正在移动的跟随车索引(默认值是0，0是头车)
    VecPoseStampPtr last_follow_poses_ptr_;
};

    
} // namespace autofleet

#endif // STATE_HPP