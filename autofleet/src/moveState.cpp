#include "autofleet/moveState.hpp"
#include "algorithm"

namespace autofleet
{
const float max_tolerate_dist_error = 10.0;

//响应行为树tick
BT::NodeStatus MoveState::tick()
{
  auto node = GetNodeSharePtr();
  LOG_OUT_INFO(node->get_logger(), "MoveState tick");
  if(is_first_tick_)
  {
    auto info = getInput<VecPoseStampPtr>("input_last_follow_poses_ptr");
    if (!info)
    {
      LOG_OUT_ERROR(node->get_logger(), "no last_follow_poses_ptr %s", info.error().c_str());
      return BT::NodeStatus::FAILURE;
    }

    last_follow_poses_ptr_ = info.value();
    LOG_OUT_INFO(node->get_logger(), "last_follow_poses size: %d", last_follow_poses_ptr_->size());

    is_first_tick_ = false;
    node->SendGoal(robot_infos_[0].robot_name, *(node->GetTargetPose())); // 向头车发送目标点
  }
  else
  {
    //查看头车是否到达目标点
    if(FollowMovedEnd(*node->GetTargetPose(), robot_infos_[0].robot_name))
    {
      return BT::NodeStatus::SUCCESS;
    }
    // 观察编队状态
    float dist_error = compute_max_dist_error();
    // 控制头车速度
    compute_head_vel_rate(dist_error);
    // 向后车发送目标点
    send_follow_goal();
  }
  
  return BT::NodeStatus::RUNNING;
}

void MoveState::SetFollowPose(VecPoseStampPtr follow_poses_ptr)
{
  last_follow_poses_ptr_ = follow_poses_ptr;
}

void MoveState::send_follow_goal()
{
  auto follow_pose = ComputeFollowPose();
  auto node = GetNodeSharePtr();
  for(int i = 1; i < robot_infos_.size(); i++)
  {
    node->SendGoal(robot_infos_[i].robot_name, follow_pose->at(i));
  }
}

void MoveState::compute_head_vel_rate(float dist_error)
{
  auto node = GetNodeSharePtr();
  if(dist_error > max_tolerate_dist_error)
  {
    // 规划出了问题，需要报警且重新控制编队
    LOG_OUT_ERROR(node->get_logger(), "dist_error > max_tolerate_dist_error");
    return;
  }
  std_msgs::msg::Float32 head_vel_rate;
  head_vel_rate.data = std::max(1.0 - (dist_error / max_tolerate_dist_error), 0.0);

  head_vel_rate_pub_->publish(head_vel_rate);
}

float MoveState::compute_max_dist_error()
{
  auto node = GetNodeSharePtr();
  float head_dist_error = 0.0;
  compute_controller_dist_error(0, head_dist_error);

  float max_dist_error = head_dist_error;
  for(int i = 1; i < robot_infos_.size(); i++)
  {
    float dist_error = 0.0;
    compute_controller_dist_error(i, dist_error);
    if(dist_error > max_dist_error)
    {
      max_dist_error = dist_error;
    }
  }

  return max_dist_error - head_dist_error;
}

bool MoveState::compute_controller_dist_error(int robot_index, float& dist_error)
{
  PoseStamp out_pose;

  if(TransformPose(last_follow_poses_ptr_->at(robot_index), out_pose, robot_infos_[robot_index].robot_name + "_base_link"))
  {
    // 只需要计算x轴上的前向误差
    dist_error = std::max(0.0, out_pose.pose.position.x);
    return true;
  }
  return false;
}
    
} // namespace autofleet

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autofleet::MoveState>("MoveState");
}
