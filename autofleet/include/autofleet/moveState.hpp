#ifndef MOVE_STATE_HPP
#define MOVE_STATE_HPP

#include "state.hpp"
#include "autofleet_node.hpp"
#include "std_msgs/msg/float32.hpp"
namespace autofleet
{
class MoveState : public State
{   
public:
    MoveState(
    const std::string& xml_tag_name, const BT::NodeConfiguration& conf) 
    : State(xml_tag_name, conf){
        auto node = GetNodeSharePtr();
        if(node == nullptr) return;
        head_vel_rate_pub = node->create_publisher<std_msgs::msg::Float32>(robot_infos_[0].robot_name + "/vel_rate", 10);
    }

    virtual ~MoveState() = default;

    virtual BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return BT::PortsList{
            BT::InputPort<VecPoseStampPtr>("input_last_follow_poses_ptr")
        };
    }

    virtual void SetFollowPose(VecPoseStampPtr follow_poses_ptr) override;

private:
    void compute_head_vel_rate(float dist_error); // 计算并发送下一次控制时头车的速度比率
    float compute_max_dist_error();
    bool compute_controller_dist_error(int robot_index, float& dist_error);
    void send_follow_goal(); // 发送跟随目标

private:
    bool is_first_tick = true;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr head_vel_rate_pub = nullptr;
    VecPoseStampPtr last_follow_poses_ptr_ = nullptr;
};

    
} // namespace autofleet

#endif // STATE_HPP