#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class TwistToTwistStamped : public rclcpp::Node
{
public:
    TwistToTwistStamped()
    : Node("twist_to_twist_stamped")
    {
        // 创建订阅器，订阅原始的 Twist 消息
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&TwistToTwistStamped::twistCallback, this, std::placeholders::_1));
        
        // 创建发布器，发布 TwistStamped 消息
        twist_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel_stamped", 10);
        
        RCLCPP_INFO(this->get_logger(), "Twist to TwistStamped converter started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /cmd_vel");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /cmd_vel_stamped");
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 创建 TwistStamped 消息
        auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
        
        // 设置 header
        twist_stamped_msg.header.stamp = this->now();
        twist_stamped_msg.header.frame_id = "base_link"; // 可根据需要修改坐标系
        
        // 复制 Twist 数据
        twist_stamped_msg.twist = *msg;
        
        // 发布转换后的消息
        twist_stamped_pub_->publish(twist_stamped_msg);
        
        // 可选：打印调试信息
        RCLCPP_DEBUG(this->get_logger(), 
                    "Converted Twist to TwistStamped: linear.x=%.2f, angular.z=%.2f",
                    msg->linear.x, msg->angular.z);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistToTwistStamped>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}