#ifndef BREAK_TEAM_STATE_HPP
#define BREAK_TEAM_STATE_HPP

#include "state.hpp"
#include "autofleet_node.hpp"
namespace autofleet
{
class BreakTeamState : public State
{
private:
    
public:
    BreakTeamState(
        const std::string& xml_tag_name, const BT::NodeConfiguration& conf);
    virtual ~BreakTeamState() = default;
    
    static BT::PortsList providedPorts()
    {
        return BT::PortsList{};
    }

    virtual BT::NodeStatus tick() override;

    int follow_index_ = 0;
    std::vector<int> has_send_goal_;
    VecPoseStampPtr follow_pose_;
};

    
} // namespace autofleet

#endif // STATE_HPP