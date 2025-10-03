#include "rclcpp/rclcpp.hpp"
#include "rclcpp/graph_listener.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <chrono>

class InitPoseNode : public rclcpp::Node{
public:
    InitPoseNode() : Node("init_pose_node") {

        this->declare_parameter("init_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter("robot_name", "robot1");

        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);

        auto robot_name = this->get_parameter("robot_name").as_string();

        RCLCPP_INFO(this->get_logger(), "%s init_pose_node 启动中", robot_name.c_str());

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
            [&, robot_name](){
                auto node_name_list = this->get_node_graph_interface()->get_node_names();
                std::string target_node_name = "/" + robot_name + "/amcl";
                for(auto node_name : node_name_list){
                    if(node_name == target_node_name){
                        RCLCPP_INFO(this->get_logger(), "amcl node 已启动");
                        timer_->cancel();
                        this->SendInitPose();
                        return;
                    }
                }
                RCLCPP_INFO(this->get_logger(), "amcl node 未启动");
            });

        auto init_pose = this->get_parameter("init_pose").as_double_array();
        RCLCPP_INFO(this->get_logger(), "位姿信息已发布(%f, %f, %f)", init_pose[0], init_pose[1],init_pose[2]);
    }

private:
    void SendInitPose(){
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";

        auto init_pose = this->get_parameter("init_pose").as_double_array();
        msg.pose.pose.position.x = init_pose[0];
        msg.pose.pose.position.y = init_pose[1];
        msg.pose.pose.position.z = init_pose[2];
        tf2::Quaternion q;
        q.setRPY(init_pose[3], init_pose[4], init_pose[5]);  // 滚转、俯仰、偏航角（弧度）
        msg.pose.pose.orientation = tf2::toMsg(q);

        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "位姿信息已发布(%f, %f, %f)", init_pose[0], init_pose[1],init_pose[2]);
    }

private:
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitPoseNode>();
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(node);
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
