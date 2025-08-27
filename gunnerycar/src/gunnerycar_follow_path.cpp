#include "gunnerycar_follow_path.h"

FollowPath::FollowPath() : Node("follow_path_node")
{
    this->declare_parameter("waypoints", std::vector<double>{0, 0, 0, 1, 1, 1, 2, 2, 2});
    RCLCPP_INFO(this->get_logger(), "FollowPath node is created");
    speak_client_ = this->create_client<gunnerycar::srv::Speak>("speak");
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

    if(!speak_client_->wait_for_service(std::chrono::milliseconds(50)))
    {
        RCLCPP_ERROR(this->get_logger(), "Speak service not available");
        return;
    }

    if(is_send_goal_success_) return;

    if(waypoints_.empty())
    {
        auto param = this->get_parameter("waypoints").as_double_array();
        for(int i = 0; i < int(param.size()) / 3; i++)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = param[i * 3 + 0];
            pose.pose.position.y = param[i * 3 + 1];
            pose.pose.position.z = 0;
            RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
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
    RCLCPP_INFO(this->get_logger(), "获得信息 Received feedback: %d", feedback->current_waypoint);
    if(feedback->current_waypoint && this->current_point != feedback->current_waypoint)
    {
        this->current_point = feedback->current_waypoint;
        auto request = std::make_shared<gunnerycar::srv::Speak::Request>();
        request->content = "到达第" + std::to_string(this->current_point) + "个点";
        speak_client_->async_send_request(request, [](std::shared_future<std::shared_ptr<gunnerycar::srv::Speak::Response>> future){
            auto response = future.get();
            if (!response) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Null response");
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speak response: %d", response->result);
            }
        });
        
    }
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