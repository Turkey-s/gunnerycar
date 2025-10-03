#include <chrono>
#include <memory>
#include "autofleet/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace cb_group_demo
{
class DemoNode : public rclcpp::Node
{
public:
    DemoNode() : Node("client_node")
    {
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_ptr_ = this->create_client<std_srvs::srv::Trigger>("/robot1/lifecycle_manager_navigation/is_active", rmw_qos_profile_services_default,
                                                                client_cb_group_);
        timer_ptr_ = this->create_wall_timer(1s, std::bind(&DemoNode::timer_callback, this),
                                            timer_cb_group_);
    }

private:
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;

    void timer_callback()
    {
        char *ptr="linuxers.cn";
        *ptr=0;
        RCLCPP_INFO(this->get_logger(), "%s Sending request %s", autofleet::BLUE, autofleet::RESET);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result_future = client_ptr_->async_send_request(request);
        std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
        if (status == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Received response");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Request timeout");
        }
    }
};  // class DemoNode
}   // namespace cb_group_demo

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<cb_group_demo::DemoNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);

    RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();
    RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
    return 0;
}