#ifndef MOVE_STATE_HPP
#define MOVE_STATE_HPP

#include "state.hpp"
#include "autofleet_node.hpp"
namespace autofleet
{
class MoveState : public State
{
private:
    
public:
    MoveState(
        const std::string& xml_tag_name, const BT::NodeConfiguration& conf) 
        : State(xml_tag_name, conf){}
    virtual ~MoveState() = default;

    void ComputeHeadVelRate(); // 计算下一次控制时头车的速度
    virtual BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return BT::PortsList{

        };
    }
};

    
} // namespace autofleet

#endif // STATE_HPP