#ifndef NAV2_CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_custom_controller{
class Nav2CustomController : public nav2_core::Controller{
public:
  Nav2CustomController() = default;
  ~Nav2CustomController() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &) override;
  void cleanup() override;
  
  void activate() override;

  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;
private:
  geometry_msgs::msg::PoseStamped
    getNearestTargetPose(const geometry_msgs::msg::PoseStamped& cur_pose);

  double calculateAngleDiff(const geometry_msgs::msg::PoseStamped& cur_pose,
                              const geometry_msgs::msg::PoseStamped& target_pose);
private:
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D* costmap_;
  nav_msgs::msg::Path global_plan_;
};
}

#endif
