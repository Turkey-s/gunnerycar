#include "autofleet/formTeamState.hpp"

namespace autofleet
{

BT::NodeStatus FormTeamState::tick()
{
  // 计算出每个车要跟随的位姿
  auto follow_path = ComputeFollowPose();

  if(follow_path->size() != robot_infos_.size() - 1)
  {
    std::cout << "FormTeamState ComputeFollowPose failed!" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if(m_stage == STAGE_NONE) m_stage = STAGE_HEAD_MOVE;
  auto node = GetNodeSharePtr();

  switch (m_stage)
  {
  case STAGE_HEAD_MOVE:
    //编队中只有头车，那么直接结束此阶段
    if(follow_path->size() == 0) return BT::NodeStatus::SUCCESS;

    for(int index = 1; index < robot_infos_.size(); index++)
    {
      if(!FollowCanMove(follow_path->at(index - 1), robot_infos_[index].robot_name)) return BT::NodeStatus::RUNNING;
    }

    m_stage = STAGE_FOLLOW_MOVE;
    node->CancelGoal(); // 让头车原地待命
    //跟随车启动TODO
    m_follow_moving_index = 1; // index = 1的车开始跟随
    return BT::NodeStatus::RUNNING;

    break;
  case STAGE_FOLLOW_MOVE:
    if(FollowMovedEnd(follow_path->at(m_follow_moving_index - 1), robot_infos_[m_follow_moving_index].robot_name))
    {
      m_follow_moving_index++;
      if(m_follow_moving_index == robot_infos_.size())
      {
        m_stage = STAGE_NONE;
        return BT::NodeStatus::SUCCESS;
      }
      //下一个编号的跟随车启动TODO
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
    break;

  default:
    break;
  }

  // 如果走到这里，说明有问题
  return BT::NodeStatus::FAILURE;
}



bool FormTeamState::FollowCanMove(PoseStamp& follow_pose, std::string robot_name)
{
  PoseStamp out_pose;
  bool b_trans = TransformPose(follow_pose, out_pose, robot_name + "_base_link");
  if(!b_trans){
    std::cout << "FormTeamState TransformPose failed!" << std::endl;
    return false;
  }

  float theta = atan2(out_pose.pose.position.y, out_pose.pose.position.x);
  if(fabs(theta) > max_turn_angle) return false;

  return true;
}

bool FormTeamState::FollowMovedEnd(PoseStamp& follow_pose, std::string robot_name) // 判断某个车是否到达目标点
{
  PoseStamp out_pose;
  bool b_trans = TransformPose(follow_pose, out_pose, robot_name + "_base_link");
  if(!b_trans){
    std::cout << "FormTeamState TransformPose failed!" << std::endl;
    return false;
  }

  // 判断阈值
  if(std::hypot(out_pose.pose.position.x, out_pose.pose.position.y) < path_pose_interval) return true;

  return false;
}
    
} // namespace autofleet

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autofleet::FormTeamState>("FormTeamState");
}
