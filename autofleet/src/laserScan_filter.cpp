#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <limits>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LaserFilter : public rclcpp::Node {
private:
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> filtered_scan_pub_;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;  
    rclcpp::CallbackGroup::SharedPtr client_cb_group_; // 回调组
    std::vector<std::string> robots_name;

public:
    LaserFilter() : Node("laser_filter") {
        // 声明参数
        declare_parameter("robots_name", std::vector<std::string>{"robot1", "robot2", "robot3"});
        robots_name = this->get_parameter("robots_name").as_string_array();
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions options;
        options.callback_group = client_cb_group_;
        for (const auto& robot_name : robots_name) {
            if(robot_name == "robot1") continue;
            laser_sub_[robot_name] = this->create_subscription<sensor_msgs::msg::LaserScan>(
            robot_name + "/scan", 10, [this, robot_name](const sensor_msgs::msg::LaserScan::SharedPtr scan){
                this->laserCallback(scan, robot_name);
            }, options);
            filtered_scan_pub_[robot_name] = this->create_publisher<sensor_msgs::msg::LaserScan>(robot_name + "/filtered_scan", 10);
        }
        //初始化tf树
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Laser filter node initialized");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, const std::string robot_name) {
        //检查tf变换
        std::string front_robot_name = "robot1";
        for(int i = 0; i < robots_name.size(); i++)
        {
            if(robots_name[i] == robot_name)
            {
                front_robot_name = robots_name[i-1];
                break;
            }
        }
        try{
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer_->lookupTransform(robot_name + "_base_link", front_robot_name + "_base_link", tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(), "Transform from %s to %s is available, %f ,%f", robot_name.c_str(),front_robot_name.c_str(),transformStamped.transform.translation.x, transformStamped.transform.translation.y);
            return;
        }catch(tf2::TransformException &ex){
            RCLCPP_WARN(this->get_logger(), "注意Could not transform %s to %s :...: %s", robot_name.c_str(), front_robot_name.c_str(), ex.what());
            return;
        }
        
        auto filtered_scan = *scan;
        
        // 获取参数
        double front_angle_min = this->get_parameter("front_angle_min").as_double();
        double front_angle_max = this->get_parameter("front_angle_max").as_double();
        
        // 遍历所有激光点
        for(size_t i = 0; i < scan->ranges.size(); ++i) {
            // 计算当前点的角度
            double angle = scan->angle_min + i * scan->angle_increment;
            
            // 如果角度在前方范围内，将该点设为无效值
            if(angle >= front_angle_min && angle <= front_angle_max) {
                filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }
        
        filtered_scan_pub_[robot_name]->publish(filtered_scan);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserFilter>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}