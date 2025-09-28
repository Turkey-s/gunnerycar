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
        const std::string& xml_tag_name, const BT::NodeConfiguration& conf) 
        : State(xml_tag_name, conf){}
    virtual ~BreakTeamState() = default;
    
    static BT::PortsList providedPorts()
    {
        return BT::PortsList{};
    }

    virtual BT::NodeStatus tick() override;
};

    
} // namespace autofleet

#endif // STATE_HPP