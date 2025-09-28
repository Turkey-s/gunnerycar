#include <chrono>
#include <functional>
#include "autofleet/autofleet_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autofleet/breakTeamState.hpp"
#include "autofleet/formTeamState.hpp"
#include "autofleet/moveState.hpp"
#include "iostream"
#include <fstream>

namespace autofleet
{
const float first_lookaheading_angle = 0.0;
using namespace std::chrono_literals;
AutofleetMgrNode::AutofleetMgrNode() : Node("autofleet_node")
{
  RCLCPP_INFO(get_logger(), "Creating autofleet node");
  declare_parameter("bt_xml_file", "bt_autofleet.xml");
  declare_parameter("plugins", std::vector<std::string>{"FormTeamState", "MoveState", "BreakTeamState"});
  declare_parameter("target_pose", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter("robots_name", std::vector<std::string>{"robot1", "robot2", "robot3"});

  // 初始化成员变量
  head_lookhead_path_ = std::make_shared<nav_msgs::msg::Path>();
  head_lookhead_path_->header.frame_id = "map";

  // 初始化目标点
  auto target_pose_param = this->get_parameter("target_pose").as_double_array();
  target_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
  target_pose_->position.x = target_pose_param[0];
  target_pose_->position.y = target_pose_param[1];
  target_pose_->orientation.z = target_pose_param[2];
  target_pose_->orientation.w = 1.0;

  // 初始化机器人列表
  auto robots_name = this->get_parameter("robots_name").as_string_array();
  head_robot_name_ = robots_name[0];
  for (const auto & robot_name : robots_name) {
    declare_parameter(robot_name, std::vector<double>{0.0, 0.0});
    auto pose = this->get_parameter(robot_name).as_double_array();
    robots_info_.push_back(RobotInfo(robot_name, pose)); 
  }

  // 排序
  std::sort(robots_info_.begin(), robots_info_.end(), [&](const RobotInfo & a, const RobotInfo & b) {
    if(a.relative_pose.x < b.relative_pose.x) {
      return true;
    }
    else if(a.relative_pose.x == b.relative_pose.x && a.relative_pose.y < b.relative_pose.y) {
      return true;
    }
    return false;
  });

  //初始化tf树
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化导航客户端
  client_ = rclcpp_action::create_client<NavigateToPose>(this, head_robot_name_ + "/navigate_to_pose");

  // 初始化订阅者
  head_lookahead_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    head_robot_name_ + "/lookahead_point",
    1,                 
    std::bind(&AutofleetMgrNode::lookahead_callback, this, std::placeholders::_1)
  );

  //初始化发布者
  path_pub_ = create_publisher<nav_msgs::msg::Path>("head_path", 1);
}

void AutofleetMgrNode::Run()
{
  CreateTree();
  // 初始化定时器
  timer_ = this->create_wall_timer(1s, std::bind(&AutofleetMgrNode::tf_timer_callback, this));
}

geometry_msgs::msg::Pose::SharedPtr AutofleetMgrNode::GetTargetPose()
{
  return target_pose_;
}

std::shared_ptr<tf2_ros::Buffer> AutofleetMgrNode::GetTfBuffer()
{
  return tf_buffer_;
}

nav_msgs::msg::Path::SharedPtr AutofleetMgrNode::GetHeadPath()
{
  return head_lookhead_path_;
}

std::vector<RobotInfo> AutofleetMgrNode::GetRobotsInfo()
{
  return robots_info_;
}

void AutofleetMgrNode::CreateTree()
{
#ifdef TESTING
  RCLCPP_INFO(this->get_logger(), "注意Creating tree");
#endif
  std::vector<std::string> plugin_libraries = get_parameter("plugins").as_string_array();
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }

  auto bt_xml_file = this->get_parameter("bt_xml_file").as_string();
  auto package_share_dir = ament_index_cpp::get_package_share_directory("autofleet");
  bt_xml_file = package_share_dir + "/config/" + bt_xml_file;

  blackboard_ = BT::Blackboard::create();
  blackboard_->set<AutofleetMgrNode::WeakPtr>("node",weak_from_this());
  
  tree_ = factory_.createTreeFromFile(bt_xml_file, blackboard_);
}

void AutofleetMgrNode::SendGoal()
{
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    
    RCLCPP_INFO(this->get_logger(), "目标点参数：%f, %f, %f", target_pose_->position.x, target_pose_->position.y, target_pose_->orientation.z);

    goal_msg.pose.pose.position = target_pose_->position;
    goal_msg.pose.pose.orientation = target_pose_->orientation;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&AutofleetMgrNode::goal_responce_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&AutofleetMgrNode::result_callback, this, std::placeholders::_1);
    this->client_->async_send_goal(goal_msg, send_goal_options);
}

void AutofleetMgrNode::CancelGoal()
{
    if (goal_handle_) {
        this->client_->async_cancel_goal(goal_handle_);
        goal_handle_ = nullptr;
    }
}

void AutofleetMgrNode::goal_responce_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        goal_handle_ = goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void AutofleetMgrNode::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
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

void AutofleetMgrNode::lookahead_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped out_pose = *msg;
  if(msg->header.frame_id != "map"){
    try{
      tf_buffer_->transform(*msg, out_pose, "map", tf2::durationFromSec(1.0));
      out_pose.header.frame_id = "map";
    }
    catch(tf2::TransformException &ex){
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s :...: %s", msg->header.frame_id.c_str(), "map", ex.what());
    }
  }

  if(head_lookhead_path_->poses.size() > 0){
    auto& last_pose = head_lookhead_path_->poses.back();
    // 去除太密集的点
    if(std::pow(last_pose.pose.position.x - out_pose.pose.position.x, 2) + std::pow(last_pose.pose.position.y - out_pose.pose.position.y, 2) < path_pose_interval * path_pose_interval){
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "lookahead point: %f, %f, %f", out_pose.pose.position.x, out_pose.pose.position.y, out_pose.pose.orientation.z);

  /*为path中的每个点添加朝向*/
  if(head_lookhead_path_->poses.size() > 0)
  {
    auto& last_pose = head_lookhead_path_->poses.back();
    last_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), atan2(out_pose.pose.position.y - last_pose.pose.position.y, out_pose.pose.position.x - last_pose.pose.position.x)));
    if(head_lookhead_path_->poses.size() > 1)
    {
      auto& last_second_pose = head_lookhead_path_->poses[head_lookhead_path_->poses.size() - 2];
      float yaw = atan2(out_pose.pose.position.y - last_second_pose.pose.position.y, out_pose.pose.position.x - last_second_pose.pose.position.x);
      last_pose.pose.orientation.z = yaw;
    }
  }
  else
  {
    out_pose.pose.orientation.z = first_lookaheading_angle; // 默认第一个点的朝向，这个需要保证一定是正确的，可以选择读参数，或根据全局定位的结果来赋值
  }

  head_lookhead_path_->poses.push_back(out_pose);

  tree_.tickRoot(); // 在收到头车点的目标点后，执行一次行为树

  path_pub_->publish(*head_lookhead_path_); // for visualization
}

void AutofleetMgrNode::tf_timer_callback(){
  for(auto& robot : robots_info_){
    if(robot.is_prepared) continue;

    try{
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped = tf_buffer_->lookupTransform("map", robot.robot_name + "_base_link", tf2::TimePointZero);
      robot.is_prepared = true;
    }catch(tf2::TransformException &ex){
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s :...: %s", "map", robot.robot_name.c_str(), ex.what());
      return;
    }
  }

  // 所有机器人已就位
  timer_->cancel();
  SendGoal();
  // tree_.tickRootWhileRunning(std::chrono::milliseconds(30));
}


AutofleetMgrNode::~AutofleetMgrNode()
{
  RCLCPP_INFO(this->get_logger(), "注意AutofleetMgrNode::~AutofleetMgrNode()");
#ifdef TESTING
  WriteFile();
#endif
}

void AutofleetMgrNode::WriteFile()
{
  std::ofstream outFile("/home/syl/gunnerycar_ws/src/autofleet/head_lookahead_path.txt");

  // 检查文件是否成功打开
  if (!outFile.is_open()) {
      std::cerr << "无法打开文件进行写入" << std::endl;
      return;
  }

  // 遍历容器并将每个元素写入文件
  for (const auto& path : head_lookhead_path_->poses) {
      outFile << std::to_string(path.pose.position.x) + " "
               + std::to_string(path.pose.position.y) + " "
               + std::to_string(path.pose.orientation.z) + "\n"
      << std::endl;
  }

  // 关闭文件流（虽然在这个例子中不是必须的，因为 outFile 会在作用域结束时自动关闭）
  outFile.close();

  std::cout << "数据已成功保存到 head_lookahead_path.txt" << std::endl;
}

}