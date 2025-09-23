#include "autofleet/breakTeamState.hpp"

namespace autofleet
{
void BreakTeamState::ComputeHeadVel(){
    
}

void BreakTeamState::ComputeFollowPose(){
    
}

void BreakTeamState::ComputeFollowVel(){
    
}
    
} // namespace autofleet

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autofleet::BreakTeamState>("BreakTeamState");
}