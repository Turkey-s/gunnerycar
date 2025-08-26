#include "rclcpp/rclcpp.hpp"
#include "rclcpp/graph_listener.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <chrono>

class InitPoseNode : public rclcpp::Node{
public:
    InitPoseNode() : Node("init_pose_node") {

        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
            [&](){
                auto node_name_list = this->get_node_graph_interface()->get_node_names();
                std::string target_node_name = "/amcl";
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
    }

private:
    void SendInitPose(){
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // 滚转、俯仰、偏航角（弧度）
        msg.pose.pose.orientation = tf2::toMsg(q);

        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "位姿信息已发布");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
