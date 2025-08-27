#include "gunnerycar_follow_path.h"

FollowPath::FollowPath() : Node("gunnerycar_follow_path")
{
    follow_path_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "FollowWaypoints");
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FollowPath::onTimerCallback, this));
}

void FollowPath::onTimerCallback()
{
    if (!follow_path_client_->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        return;
    }

    if(is_send_goal_success_) return;

    if(waypoints_.empty())
    {
        for(int x = 0; x < 3; x++)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = x;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0;

            waypoints_.push_back(pose);
        }
    }

    auto msg = std::make_shared<NavigateThroughPoses::Goal>();
    msg->poses = waypoints_;

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&FollowPath::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&FollowPath::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&FollowPath::resultCallback, this, std::placeholders::_1);


    follow_path_client_->async_send_goal(*msg, send_goal_options);
}

void FollowPath::goalResponseCallback(std::shared_future<GoalHandleNavigateThroughPoses::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        is_send_goal_success_ = true;
    }
}

void FollowPath::feedbackCallback(GoalHandleNavigateThroughPoses::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->current_waypoint);
}

void FollowPath::resultCallback(const GoalHandleNavigateThroughPoses::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal was SUCCESS");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    }
}