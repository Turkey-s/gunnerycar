#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gunnerycar/srv/speak.hpp"

#include <vector>

class FollowPath : public rclcpp::Node{
public:
    using NavigateThroughPoses = nav2_msgs::action::FollowWaypoints;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
    FollowPath();
private:
    void onTimerCallback();
    void goalResponseCallback(std::shared_future<GoalHandleNavigateThroughPoses::SharedPtr> future);
    void feedbackCallback(GoalHandleNavigateThroughPoses::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateThroughPoses::WrappedResult & result);
    
private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr follow_path_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    bool is_send_goal_success_ = false;
    rclcpp::Client<gunnerycar::srv::Speak>::SharedPtr speak_client_;
    uint32_t current_point = -1;
};