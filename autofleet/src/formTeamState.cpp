#include "autofleet/formTeamState.hpp"

namespace autofleet
{

BT::NodeStatus FormTeamState::tick()
{
  std::cout << "FormTeamState::tick enter" << std::endl;

  //编队中只有头车，那么直接结束此阶段
  if(robot_infos_.size() == 1) return BT::NodeStatus::SUCCESS;

  // 计算出每个车要跟随的位姿
  auto follow_path = ComputeFollowPose();

  if(follow_path == nullptr)
  {
    return BT::NodeStatus::RUNNING;
  }

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
    LOG_OUT_INFO(node->get_logger(), "");

    for(int index = 1; index < robot_infos_.size(); index++)
    {
      if(!FollowCanMove(follow_path->at(index - 1), robot_infos_[index].robot_name)) return BT::NodeStatus::RUNNING;
    }

    m_stage = STAGE_FOLLOW_MOVE;
    node->CancelGoal(robot_infos_[0].robot_name); // 让头车原地待命
    node->SendGoal(robot_infos_[1].robot_name, follow_path->at(0));
    m_follow_moving_index = 1; // index = 1的车开始跟随
    LOG_OUT_INFO(node->get_logger(), "");
    return BT::NodeStatus::RUNNING;

    break;
  case STAGE_FOLLOW_MOVE:
    if(FollowMovedEnd(follow_path->at(m_follow_moving_index - 1), robot_infos_[m_follow_moving_index].robot_name))
    {
      LOG_OUT_INFO(node->get_logger(), "%s followed end", robot_infos_[m_follow_moving_index].robot_name.c_str());
      node->CancelGoal(robot_infos_[m_follow_moving_index].robot_name);
      m_follow_moving_index++;
      if(m_follow_moving_index == robot_infos_.size())
      {
        m_stage = STAGE_NONE;
        return BT::NodeStatus::SUCCESS;
      }
      //下一个编号的跟随车启动
      node->SendGoal(robot_infos_[1].robot_name, follow_path->at(m_follow_moving_index - 1));
    }
    else
    {
      LOG_OUT_INFO(node->get_logger(), "");
      return BT::NodeStatus::RUNNING;
    }
    break;

  default:
    break;
  }
  LOG_OUT_INFO(node->get_logger(), "");
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
