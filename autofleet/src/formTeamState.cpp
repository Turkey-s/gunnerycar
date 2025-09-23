#include "autofleet/formTeamState.hpp"

namespace autofleet
{
void FormTeamState::ComputeHeadVel(){
    
}

void FormTeamState::ComputeFollowPose(){
    
}

void FormTeamState::ComputeFollowVel(){
    
}
    
} // namespace autofleet

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autofleet::FormTeamState>("FormTeamState");
}
