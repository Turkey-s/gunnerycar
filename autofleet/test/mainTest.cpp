#include <chrono>
#include <memory>
#include "autofleet/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

namespace cb_group_demo
{
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
class DemoNode : public rclcpp::Node
{
public:
    DemoNode() : Node("client_node")
    {
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_ptr_ = this->create_client<std_srvs::srv::Trigger>("/robot1/lifecycle_manager_navigation/is_active", rmw_qos_profile_services_default,
                                                                client_cb_group_);
        timer_ptr_ = this->create_wall_timer(1s, std::bind(&DemoNode::timer_callback, this),
                                            timer_cb_group_);
        action_nav_to_goal_ = rclcpp_action::create_client<NavigateToPose>(this, "robot2/navigate_to_pose", client_cb_group_);
        goals = {{0.43,-0.05, 0.0}, {0.776633, -0.034255, -0.011282}, 
        {1.474498, -0.042129, -0.011282}, {1.852166, -0.046389, -0.011282},
        {2.200964, -0.050325, -0.011282}, {2.776441, -0.060620, -0.055479},
        {3.124068, -0.091033, -0.087265},{3.471695, -0.121446, -0.08985}, {10.819323, -0.151860, -0.08985}};
    }

    void SendGoal(geometry_msgs::msg::PoseStamped target_pose);
    void goal_responce_callback(std::string robot_name, std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);
    void result_callback(std::string robot_name, const GoalHandleNavigateToPose::WrappedResult & result);

private:
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_nav_to_goal_;
    std::vector<std::vector<float> > goals; // 目标点
    int index = 0;

    void timer_callback()
    {
        if(index >= goals.size()) {
            return;
        }

        auto pose = geometry_msgs::msg::PoseStamped();
        pose.pose.position.x = goals[index][0];
        pose.pose.position.y = goals[index][1];
        pose.pose.orientation.z = goals[index][2];

        SendGoal(pose);
        index++;
    }
};  // class DemoNode

void DemoNode::goal_responce_callback(std::string robot_name, std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server %s", robot_name.c_str());
    } else {
        LOG_OUT_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
}

void DemoNode::result_callback(std::string robot_name, const GoalHandleNavigateToPose::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "%s Goal 被放弃", robot_name.c_str());
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "%s Goal was canceled", robot_name.c_str());
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "%s Unknown result code", robot_name.c_str());
    }
}

void DemoNode::SendGoal(geometry_msgs::msg::PoseStamped target_pose)
{
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = target_pose;
    goal_msg.pose.header.stamp = this->now();
    
    RCLCPP_INFO(get_logger(), "目标点参数：%f, %f, %f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.orientation.z);
    auto send_goal_options_ = std::make_shared<rclcpp_action::Client<NavigateToPose>::SendGoalOptions>();
    send_goal_options_->goal_response_callback = std::bind(&DemoNode::goal_responce_callback, this,"robot2", std::placeholders::_1);
    send_goal_options_->result_callback = std::bind(&DemoNode::result_callback, this,"robot2", std::placeholders::_1);

    action_nav_to_goal_->async_send_goal(goal_msg, *send_goal_options_);
}

}   // namespace cb_group_demo

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<cb_group_demo::DemoNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);

    RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
    return 0;
}