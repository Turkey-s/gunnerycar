#ifndef STATE_H
#define STATE_H

#include "autofleet_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace autofleet
{
using PoseStamp = geometry_msgs::msg::PoseStamped;
using VecPoseStampPtr = std::shared_ptr<std::vector<PoseStamp> >;

const float head_xy_goal_tolerance = 0.7;
const float follow_xy_goal_tolerance = 0.5;
class State : public BT::ActionNodeBase
{
public:
    State() = delete;

    State(
        const std::string& xml_tag_name, const BT::NodeConfiguration& conf) 
    : BT::ActionNodeBase(xml_tag_name, conf) 
    {
        xml_tag_name_ = xml_tag_name;
        weak_node_ = config().blackboard->get<AutofleetMgrNode::WeakPtr>("node");

        auto node = GetNodeSharePtr();
        if(node == nullptr) return;
        
        robot_infos_ = node->GetRobotsInfo();
    };
    virtual ~State() = default;

    virtual BT::NodeStatus tick() override
    {
        if(weak_node_.expired()) return BT::NodeStatus::FAILURE;
        auto node = weak_node_.lock();
        LOG_OUT_INFO(node->get_logger(), "tick %s", xml_tag_name_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    std::shared_ptr<AutofleetMgrNode> GetNodeSharePtr()
    {
        if(weak_node_.expired())
        {
            std::cout << "autofleet node is nullptr" << std::endl;
            return nullptr;
        }

        return std::dynamic_pointer_cast<AutofleetMgrNode>(weak_node_.lock());
    }

    void halt() override
    {
    }

    VecPoseStampPtr ComputeFollowPose();

    // 转换坐标系，将in_pose的坐标系转换到out_pose的坐标系
    bool TransformPose(const PoseStamp& in_pose, PoseStamp& out_pose, const std::string& target_frame_id);
    // 获得坐标系之前的装换关系
    bool GetTransform(const std::string& source_frame_id, const std::string& target_frame_id, geometry_msgs::msg::TransformStamped& transform);
    bool FollowMovedEnd(PoseStamp& follow_pose, std::string robot_name);

    virtual void SetFollowPose(VecPoseStampPtr follow_poses_ptr) {};

    static BT::PortsList providedPorts()
    {
        return BT::PortsList{};
    }
private:
    AutofleetMgrNode::WeakPtr weak_node_;
    std::string xml_tag_name_;

public:
    std::vector<RobotInfo> robot_infos_;
};

// 依据一个目标点，以相同的航向和偏移找到下一个目标点
void computePoseByOffset(const PoseStamp& in_pose, PoseStamp& out_pose,
                            float offset_x, float offset_y)
{
    // 前后偏移量
    out_pose.pose.position.x = in_pose.pose.position.x - offset_x * std::cos(in_pose.pose.orientation.z);
    out_pose.pose.position.y = in_pose.pose.position.y - offset_x * std::sin(in_pose.pose.orientation.z);
    out_pose.pose.orientation = in_pose.pose.orientation;

    //左右偏移量
    out_pose.pose.position.x = out_pose.pose.position.x + offset_y * std::sin(in_pose.pose.orientation.z);
    out_pose.pose.position.y = out_pose.pose.position.y - offset_y * std::cos(in_pose.pose.orientation.z);

    out_pose.header.frame_id = in_pose.header.frame_id;
}

/*暂时只支持一字型 TODO后续将支持品字形*/
//依据当前头车的目标点，计算跟随车的目标点，跟随车的目标点为头车path上的某一个点的相对位置，如有左右偏移，那就是以那一点的左右偏移量
VecPoseStampPtr State::ComputeFollowPose()
{
    auto node = GetNodeSharePtr();
    if(node == nullptr) 
    {
        LOG_OUT_ERROR(node->get_logger(), "ComputeFollowPose node is nullptr");
        return nullptr;
    }
    
    auto& head_path = node->GetHeadPath()->poses;
    
    if(robot_infos_.size() < 2 || head_path.size() == 0) 
    {
        return nullptr;
    }

    auto follow_pose = std::make_shared<std::vector<PoseStamp> >();

    follow_pose->push_back(head_path.back());
    
    // 计算跟随车的目标点，误差为0.3m
    float sum = 0.0;
    int robot_index = 1; // 跟随车索引
    for(int i = head_path.size() - 2; i >= 0; i--)
    {
        sum += std::hypot(head_path[i].pose.position.x - head_path[i + 1].pose.position.x, head_path[i].pose.position.y - head_path[i + 1].pose.position.y);
        if(sum >= robot_infos_[robot_index].relative_pose.x)
        {
            follow_pose->push_back(head_path[i]);
            LOG_OUT_INFO(node->get_logger(), "ComputeFollowPose follow_point id: %d ,pose (%f, %f)", robot_index,head_path[i].pose.position.x, head_path[i].pose.position.y);
            robot_index++;
        }

        if(robot_index == robot_infos_.size())
        {
            // head_path.assign(head_path.begin() + i, head_path.end());
            break;
        }
    }

    while(robot_index < robot_infos_.size())
    {
        PoseStamp out_pose;
        computePoseByOffset(head_path[0], out_pose, robot_infos_[robot_index].relative_pose.x - sum, robot_infos_[robot_index].relative_pose.y);
        LOG_OUT_INFO(node->get_logger(), "ComputeFollowPose %s out_pose_x:(%f, %f), head_path[0].pose:(%f, %f,%f) %s", 
        robot_infos_[robot_index].robot_name.c_str(), out_pose.pose.position.x,out_pose.pose.position.y, head_path[0].pose.position.x,head_path[0].pose.position.y, 
        head_path[0].pose.orientation.z,out_pose.header.frame_id.c_str());
        follow_pose->push_back(out_pose);
        robot_index++;
    }
    SetFollowPose(follow_pose);
    return follow_pose;
}

bool State::TransformPose(const PoseStamp& in_pose,
    PoseStamp& out_pose, const std::string& target_frame_id)
{
    auto node = GetNodeSharePtr();
    if(node == nullptr) return false;

    auto tf_ = node->GetTfBuffer();
    try {
        tf_->transform(in_pose, out_pose, target_frame_id, tf2::durationFromSec(0.5));
        out_pose.header.frame_id = target_frame_id;
        return true;
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(node->get_logger(), "Exception in transformPose: %s", ex.what());
    }
    return false;
}

bool State::GetTransform(const std::string& source_frame_id, const std::string& target_frame_id, geometry_msgs::msg::TransformStamped& transform)
{
    auto node = GetNodeSharePtr();
    if(node == nullptr) return false;

    auto tf_ = node->GetTfBuffer();
    // 监听当前时刻源坐标系到目标坐标系的坐标变换
    try {
        transform = tf_->lookupTransform(
                    source_frame_id, target_frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        // 如果坐标变换获取失败，进入异常报告
        RCLCPP_ERROR(
            node->get_logger(), "Could not transform %s to %s: %s",
            source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
        return false;
    }
    return true;
}

bool State::FollowMovedEnd(PoseStamp& follow_pose, std::string robot_name) // 判断某个车是否到达目标点
{
    auto node = GetNodeSharePtr();
    PoseStamp out_pose;
    follow_pose.header.stamp = node->now();
    bool b_trans = TransformPose(follow_pose, out_pose, robot_name + "_base_link");
    if(!b_trans){
        LOG_OUT_ERROR(node->get_logger(), "%s FollowMovedEnd TransformPose failed!", robot_name.c_str());
        return false;
    }

    // LOG_OUT_INFO(node->get_logger(), "%s follow_pose_x:%f out_pose_x:%f", robot_name.c_str(), follow_pose.pose.position.x, out_pose.pose.position.x);

    // 判断阈值
    auto distance = std::hypot(out_pose.pose.position.x, out_pose.pose.position.y);
    
    float tolerance = robot_name == robot_infos_[0].robot_name ? head_xy_goal_tolerance : follow_xy_goal_tolerance;
    // LOG_OUT_INFO(node->get_logger(), "FollowMovedEnd distance = %f, tolerance = %f", distance, tolerance);
    if(distance < tolerance) return true;

    return false;
}

} // namespace autofleet

#endif // STATE_H