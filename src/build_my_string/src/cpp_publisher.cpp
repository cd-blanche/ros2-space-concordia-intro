#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "build_my_string/msg/message.hpp"

using namespace std::chrono_literals;

class MessagePublisher : public rclcpp::Node
{
  public:
    MessagePublisher()
    : Node("message_publisher"), count_(0)
    {
      this->declare_parameter("user_message", "Hello world!");
      publisher_ = this->create_publisher<build_my_string::msg::Message>("topic", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&MessagePublisher::timer_callback, this)
      );
    }
  private:
    void timer_callback()
    {
      std::string user_message = this->get_parameter("user_message").as_string();
      auto message = build_my_string::msg::Message();
      message.message = user_message;
      this->count_++;
      RCLCPP_INFO(this->get_logger(), "Publishing: %s. [%zu]", message.message.c_str(), this->count_);
    }
    rclcpp::Publisher<build_my_string::msg::Message>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessagePublisher>());
  rclcpp::shutdown();
  return 0;
}


