#ifndef STATE_H
#define STATE_H

#include "autofleet_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace autofleet
{
using PoseStamp = geometry_msgs::msg::PoseStamped;
using VecPoseStamp = std::unique_ptr<std::vector<PoseStamp> >;
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
        RCLCPP_INFO(node->get_logger(), "tick %s", xml_tag_name_.c_str());
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

    VecPoseStamp ComputeFollowPose();

    // 转换坐标系，将in_pose的坐标系转换到out_pose的坐标系
    bool TransformPose(const PoseStamp& in_pose, PoseStamp& out_pose, const std::string& target_frame_id);
    // 获得坐标系之前的装换关系
    bool GetTransform(const std::string& source_frame_id, const std::string& target_frame_id, geometry_msgs::msg::TransformStamped& transform);

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
    out_pose.pose.position.x = in_pose.pose.position.x + offset_y * std::sin(in_pose.pose.orientation.z);
    out_pose.pose.position.y = in_pose.pose.position.y - offset_y * std::cos(in_pose.pose.orientation.z);
}

/*暂时只支持一字型 TODO后续将支持品字形*/
//依据当前头车的目标点，计算跟随车的目标点，跟随车的目标点为头车path上的某一个点的相对位置，如有左右偏移，那就是以那一点的左右偏移量
VecPoseStamp State::ComputeFollowPose()
{
    auto node = GetNodeSharePtr();
    if(node == nullptr) return VecPoseStamp();
    if(robot_infos_.size() < 2) VecPoseStamp();

    auto& head_path = node->GetHeadPath()->poses;

    auto follow_pose = VecPoseStamp();
    
    // 计算跟随车的目标点，误差为0.3m
    float sum = 0.0;
    int robot_index = 1; // 跟随车索引
    for(int i = head_path.size() - 2; i >= 0; i--)
    {
        sum += std::hypot(head_path[i].pose.position.x - head_path[i + 1].pose.position.x, head_path[i].pose.position.y - head_path[i + 1].pose.position.y);
        if(sum >= robot_infos_[robot_index].relative_pose.x)
        {
            follow_pose->push_back(head_path[i]);
            robot_index++;
        }

        if(robot_index == robot_infos_.size())
        {
            head_path.assign(head_path.begin() + i, head_path.end());
            break;
        }
    }

    while(robot_index < robot_infos_.size())
    {
        PoseStamp out_pose;
        computePoseByOffset(head_path.back(), out_pose, robot_infos_[robot_index].relative_pose.x - sum, robot_infos_[robot_index].relative_pose.y);
        follow_pose->push_back(out_pose);
        robot_index++;
    }

    return follow_pose;
}

bool State::TransformPose(const PoseStamp& in_pose,
    PoseStamp& out_pose, const std::string& target_frame_id)
{
    auto node = GetNodeSharePtr();
    if(node == nullptr) return false;

    auto tf_ = node->GetTfBuffer();
    try {
        tf_->transform(in_pose, out_pose, target_frame_id, tf2::durationFromSec(0.2));
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

} // namespace autofleet

#endif // STATE_H