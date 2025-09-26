#ifndef AUTOFLEET_NODE
#define AUTOFLEET_NODE

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace autofleet
{
struct RobotInfo{
    std::string robot_name;
    geometry_msgs::msg::Pose relative_pose; // 在编队中的相对位置
    bool is_prepared; // 是否已经准备好

    RobotInfo(std::string name,bool prepared = false) : robot_name(name){}
};

class AutofleetMgrNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    AutofleetMgrNode();
    virtual ~AutofleetMgrNode();
    void CreateTree(); // 创建行为树

    geometry_msgs::msg::Pose::SharedPtr GetTargetPose();
    std::shared_ptr<tf2_ros::Buffer> GetTfBuffer();
    nav_msgs::msg::Path::SharedPtr GetHeadPath();
    void SendGoal();
    void CancelGoal();
    void Run();

private:
    void TargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void goal_responce_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void lookahead_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void tf_timer_callback();

private:
    // 行为树相关
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::string bt_xml_filename_;
    
    // Node相关
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
    geometry_msgs::msg::Pose::SharedPtr target_pose_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ = nullptr;
    GoalHandleNavigateToPose::SharedPtr goal_handle_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr head_lookahead_sub_ = nullptr; // 订阅头车前视点

    // Test
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // 编队数据
    std::unordered_map<std::string, RobotInfo> robots_info_; // 机器人信息
    std::string head_robot_name_; // 头车名称
    nav_msgs::msg::Path::SharedPtr head_lookhead_path_; // 头车前视路径
};

}

#endif