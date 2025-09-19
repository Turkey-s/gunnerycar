#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

class NavToPoseNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavToPoseNode() : Node("init_target_node"){
        this->declare_parameter("init_target_pose", std::vector<double>{10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

        RCLCPP_INFO(this->get_logger(), "init_target_node 节点初始化...");
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&NavToPoseNode::check_prepare, this));

    }

private:
    void SendGoal()
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        
        auto param = this->get_parameter("init_target_pose").as_double_array();
        
        RCLCPP_INFO(this->get_logger(), "目标点参数：%f, %f, %f", param[0], param[1], param[5]);

        goal_msg.pose.pose.position.x = param[0];
        goal_msg.pose.pose.position.y = param[1];
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = param[5];
        goal_msg.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&NavToPoseNode::goal_responce_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&NavToPoseNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&NavToPoseNode::result_callback, this, std::placeholders::_1);


        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void check_prepare()
    {
        RCLCPP_INFO(this->get_logger(), "检查 init_target_node 先决条件...");

        if (!client_->wait_for_action_server()) {
            RCLCPP_INFO(this->get_logger(), "等待action中...");
            return;
        }

        // 设置源坐标系和目标坐标系的名称
        std::string target_frame = "map";
        std::string source_frame = "robot1_base_footprint";

        geometry_msgs::msg::TransformStamped trans;

        // 监听当前时刻源坐标系到目标坐标系的坐标变换
        try {
            trans = tf_buffer_->lookupTransform(
                        target_frame, source_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            // 如果坐标变换获取失败，进入异常报告
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
            return;
        }

        if(action_success)
        {
            timer_->cancel();
            return;
        }
        SendGoal();
    }

    void goal_responce_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            action_success = true;
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // RCLCPP_INFO(this->get_logger(), "收到反馈: 距离目标还有 %.2f 米", 
        //            feedback->distance_remaining);
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
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

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool action_success = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
