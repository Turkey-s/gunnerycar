#include "gunnerycar/nav2_custom_controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"


namespace nav2_custom_controller{
void Nav2CustomController::configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros){
    node_ = parent;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    name_ = name;
}

void Nav2CustomController::cleanup(){
    RCLCPP_INFO(node_->get_logger(), "Cleaning up controller");
}

void Nav2CustomController::activate(){
    RCLCPP_INFO(node_->get_logger(), "Activating controller");
}

void Nav2CustomController::deactivate(){
    RCLCPP_INFO(node_->get_logger(), "Deactivating controller");
}

void Nav2CustomController::setPlan(const nav_msgs::msg::Path & path)
{
    global_plan_ = path;
}

geometry_msgs::msg::PoseStamped
    Nav2CustomController::getNearestTargetPose(const geometry_msgs::msg::PoseStamped& cur_pose)
{
    if(global_plan_.poses.size() == 0)
    {
        return cur_pose;
    }

    using nav2_util::geometry_utils::euclidean_distance;
    int nearest_index = 0;
    double min_distance = euclidean_distance(cur_pose, global_plan_.poses[0]);
    for (int i = 1; i < global_plan_.poses.size(); i++) {
        double distance = euclidean_distance(cur_pose, global_plan_.poses[i]);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_index = i;
        }
    }

    global_plan_.poses.erase(global_plan_.poses.begin(), global_plan_.poses.begin() + nearest_index);

    if(global_plan_.poses.size() == 1)
    {
        return global_plan_.poses[0];
    }
    return global_plan_.poses[1];
}

double Nav2CustomController::calculateAngleDiff(const geometry_msgs::msg::PoseStamped& cur_pose,
                            const geometry_msgs::msg::PoseStamped& target_pose)
{
    float cur_yaw = tf2::getYaw(cur_pose.pose.orientation);
    float target_yaw = std::atan2(target_pose.pose.position.y - cur_pose.pose.position.y, target_pose.pose.position.x - cur_pose.pose.position.x);
    
    float diff_yaw = target_yaw - cur_yaw;
    if(diff_yaw > M_PI)
    {
        diff_yaw -= 2 * M_PI;
    }
    else if(diff_yaw < -M_PI)
    {
        diff_yaw += 2 * M_PI;
    }

    return diff_yaw;
}

geometry_msgs::msg::TwistStamped Nav2CustomController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity)
{
    if(global_plan_.poses.empty())
    {
        throw nav2_core::PlannerException("No global plan found!");
    } 

    geometry_msgs::msg::PoseStamped pose_in_global_frame;
    if(!nav2_util::getCurrentPose(pose_in_global_frame, *tf_, global_plan_.header.frame_id, pose.header.frame_id, 0.1))
    {
        throw nav2_core::PlannerException("无法将机器人姿态转换为全局计划的坐标系");
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "转换成功 x: %f, y: %f", pose_in_global_frame.pose.position.x, pose_in_global_frame.pose.position.y);
    }

    auto targetPoint = getNearestTargetPose(pose);
    float diff_yaw = calculateAngleDiff(pose, targetPoint);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose_in_global_frame.header.frame_id;
    cmd_vel.header.stamp = node_->get_clock()->now();

    if(fabs(diff_yaw) > M_PI / 10)
    {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = fabs(diff_yaw) / diff_yaw * 0.5;
    }
    else
    {
        cmd_vel.twist.linear.x = 0.2;
        cmd_vel.twist.angular.z = 0.0;
    }

    RCLCPP_INFO(node_->get_logger(), "控制器：%s 发送速度(%f,%f)",
              name_.c_str(), cmd_vel.twist.linear.x,
              cmd_vel.twist.angular.z);

    return cmd_vel;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::Nav2CustomController, nav2_core::Controller)