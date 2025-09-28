#include "autofleet/breakTeamState.hpp"

namespace autofleet
{

BT::NodeStatus BreakTeamState::tick()
{
  //获得跟随车停车目标位姿

  //启动跟随车
  return BT::NodeStatus::SUCCESS;
}
    
} // namespace autofleet

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autofleet::BreakTeamState>("BreakTeamState");
}