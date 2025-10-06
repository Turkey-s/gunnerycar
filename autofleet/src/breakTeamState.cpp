#include "autofleet/breakTeamState.hpp"

namespace autofleet
{
BreakTeamState::BreakTeamState(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) 
        : State(xml_tag_name, conf)
{
  has_send_goal_ = std::vector<int>(robot_infos_.size(), 0);
}
BT::NodeStatus BreakTeamState::tick()
{
  auto node = GetNodeSharePtr();
  LOG_OUT_INFO(node->get_logger(),"BreakTeamState tick");

  if(follow_index_ == 0)
  {
    follow_pose_ = ComputeFollowPose();
  }

  while(follow_index_ < robot_infos_.size() && 
    FollowMovedEnd(follow_pose_->at(follow_index_), robot_infos_[follow_index_].robot_name))
  {
    follow_index_++;
  }

  if(follow_index_ == robot_infos_.size())
  {
    return BT::NodeStatus::SUCCESS;
  }
  
  // if(!has_send_goal_[follow_index_])
  // {
  //   LOG_OUT_INFO(node->get_logger(),"%s BreakTeamState send goal", robot_infos_[follow_index_].robot_name.c_str());
  //   node->SendGoal(robot_infos_[follow_index_].robot_name, follow_pose_->at(follow_index_));
  //   has_send_goal_[follow_index_] = 1;
  // }

  return BT::NodeStatus::RUNNING;
}
    
} // namespace autofleet

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autofleet::BreakTeamState>("BreakTeamState");
}