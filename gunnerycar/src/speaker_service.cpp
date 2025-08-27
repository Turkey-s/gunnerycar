#include "gunnerycar/srv/speak.hpp"
#include "rclcpp/rclcpp.hpp"

class SpeakerService : public rclcpp::Node
{
public:
    SpeakerService()
    : Node("speaker_service")
    {
        this->service_ = this->create_service<gunnerycar::srv::Speak>(
            "speak",
            std::bind(&SpeakerService::handle_speak, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_speak(const std::shared_ptr<gunnerycar::srv::Speak::Request> request,
                      std::shared_ptr<gunnerycar::srv::Speak::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to speak: %s", request->content.c_str());

        response->result = true;
    }

private:
    rclcpp::Service<gunnerycar::srv::Speak>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeakerService>());
    rclcpp::shutdown();
    return 0;
}