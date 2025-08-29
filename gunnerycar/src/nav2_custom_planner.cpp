#include "nav2_custom_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_custom_planner{
    void CustomPlanner::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
        tf_ = tf;
        node_ = parent;
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
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
        //
        path.header.stamp = node_->now();
        path.header.frame_id = global_frame_;

        if(start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_){
            RCLCPP_ERROR(node_->get_logger(), "起始点和目标点必须在全局坐标系下");
            return path;
        }

        int total_points = std::hypot(goal.pose.position.x - start.pose.position.x, goal.pose.position.y - start.pose.position.y) / interpolation_resolution_;
           
        double x_increment = (goal.pose.position.x - start.pose.position.x) / total_points;
        double y_increment = (goal.pose.position.y - start.pose.position.y) / total_points;

        for(int i = 0; i <= total_points; i++){
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start.pose.position.x + x_increment * i;
            pose.pose.position.y = start.pose.position.y + y_increment * i;
            pose.header.frame_id = global_frame_;
            pose.header.stamp = node_->now();
            path.poses.push_back(pose);
        }

        for(auto & pose : path.poses){
            unsigned int mx, my;
            if(costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)){
                unsigned char cost = costmap_->getCost(mx, my);
                if(cost == nav2_costmap_2d::LETHAL_OBSTACLE)
                {
                    RCLCPP_ERROR(node_->get_logger(), "路径上存在障碍物");
                    throw nav2_core::PlannerException("路径上存在障碍物" + std::to_string(pose.pose.position.x) + "," + std::to_string(pose.pose.position.y));
                }
                
            }
        }
        path.poses.push_back(goal);
        return path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)