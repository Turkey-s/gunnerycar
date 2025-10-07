#ifndef AUTOFLEET_NODE
#define AUTOFLEET_NODE

#include "autofleet/util.hpp"

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
#include "geometry_msgs/msg/point.h"
#include "std_srvs/srv/trigger.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace autofleet
{
const float max_turn_angle = 0.72; // 最大转弯角度，弧度值
const float path_pose_interval = 0.3;
struct RobotInfo{
    std::string robot_name;
    geometry_msgs::msg::Point relative_pose; // 在编队中的相对位置
    bool is_prepared; // 是否已经准备好

    RobotInfo(std::string name, std::vector<double> pose) : robot_name(name), is_prepared(false)
    {
        relative_pose.x = pose[0];
        relative_pose.y = pose[1];
    }
};

class AutofleetMgrNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    AutofleetMgrNode();
#ifdef TESTING
    virtual ~AutofleetMgrNode();
#endif
    void CreateTree(); // 创建行为树

    geometry_msgs::msg::PoseStamped::SharedPtr GetTargetPose();
    std::shared_ptr<tf2_ros::Buffer> GetTfBuffer();
    nav_msgs::msg::Path::SharedPtr GetHeadPath();
    std::vector<RobotInfo> GetRobotsInfo();
    void SendGoal(std::string robot_name, geometry_msgs::msg::PoseStamped target_pose);
    void CancelGoal(std::string robot_name);
    void Run();

private:
    void goal_responce_callback(std::string robot_name, std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);
    void result_callback(std::string robot_name, const GoalHandleNavigateToPose::WrappedResult & result);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void lookahead_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void tf_timer_callback();

    void write_file();

private:
    // 行为树相关
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::string bt_xml_filename_;
    
    // Node相关
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
    geometry_msgs::msg::PoseStamped::SharedPtr head_target_pose_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
    std::unordered_map<std::string, rclcpp_action::Client<NavigateToPose>::SharedPtr> navigation_goal_clients_;
    std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> lifecycle_mgr_clients_;
    std::unordered_map<std::string, GoalHandleNavigateToPose::SharedPtr> goal_handle_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr head_lookahead_sub_ = nullptr; // 订阅头车前视点
    std::unordered_map<std::string, std::shared_ptr<rclcpp_action::Client<NavigateToPose>::SendGoalOptions> > send_goal_options_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr follow_pose_pub_ = nullptr; // 发布跟随点

    rclcpp::CallbackGroup::SharedPtr timer_cb_group_; //定时器互斥回调组
    rclcpp::CallbackGroup::SharedPtr client_cb_group_; // 回调组

    // Test
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // 编队数据
    std::vector<RobotInfo> robots_info_; // 机器人信息
    std::string head_robot_name_; // 头车名称
    nav_msgs::msg::Path::SharedPtr head_lookhead_path_; // 头车前视路径
};

}

#endif // AUTOFLEET_NODE