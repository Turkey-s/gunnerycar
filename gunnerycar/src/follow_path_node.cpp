// 多目标点导航
#include "gunnerycar_follow_path.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowPath>());
    rclcpp::shutdown();
    return 0;
}