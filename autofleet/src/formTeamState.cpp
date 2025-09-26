#include "autofleet/formTeamState.hpp"

namespace autofleet
{
void FormTeamState::ComputeHeadVel(){
    
}

void FormTeamState::ComputeFollowPose(){
    
}

void FormTeamState::ComputeFollowVel(){
    
}

BT::NodeStatus FormTeamState::tick()
{
  // 处理当前数据
  
  return BT::NodeStatus::SUCCESS;
}
    
} // namespace autofleet

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autofleet::FormTeamState>("FormTeamState");
}
