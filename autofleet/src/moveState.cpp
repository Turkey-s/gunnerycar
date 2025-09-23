#include "autofleet/moveState.hpp"

namespace autofleet
{
void MoveState::ComputeHeadVel(){
    
}

void MoveState::ComputeFollowPose(){
    
}

void MoveState::ComputeFollowVel(){
    
}

//响应行为树tick
BT::NodeStatus tick()
{

}
    
} // namespace autofleet

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autofleet::MoveState>("MoveState");
}
