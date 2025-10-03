#include "nav2_custom_follow_planner/nav2_custom_follow_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_custom_follow_planner{
    void CustomPlanner::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
        tf_ = tf;
        node_ = parent;
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    }

    void CustomPlanner::cleanup(){
        RCLCPP_INFO(node_->get_logger(), "正在清理类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    void CustomPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    void CustomPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    nav_msgs::msg::Path CustomPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) {
        nav_msgs::msg::Path path;
        RCLCPP_INFO(node_->get_logger(), "起始点: %s %f %f ,目标点: %s %f %f",
        start.header.frame_id.c_str(), start.pose.position.x, start.pose.position.y,
        goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        path.header.stamp = node_->now();
        path.header.frame_id = global_frame_;

        if(start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_){
            RCLCPP_ERROR(node_->get_logger(), "起始点和目标点必须在全局坐标系下");
            return path;
        }

        path.poses.push_back(goal);
        return path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_follow_planner::CustomPlanner, nav2_core::GlobalPlanner)