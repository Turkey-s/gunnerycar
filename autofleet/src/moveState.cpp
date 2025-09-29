#include "autofleet/moveState.hpp"

namespace autofleet
{
void MoveState::ComputeHeadVelRate(){

}

//响应行为树tick
BT::NodeStatus MoveState::tick()
{
  auto node = GetNodeSharePtr();

  if(is_first_tick)
    node->SendGoal(robot_infos_[0].robot_name, *(node->GetTargetPose())); // 向头车发送目标点
  
  // 观察编队状态

  // 控制头车速度

  // 向后车发送目标点
  
    return BT::NodeStatus::SUCCESS;
}
    
} // namespace autofleet

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autofleet::MoveState>("MoveState");
}
