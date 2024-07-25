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
      // Initialize custom parameter
      this->declare_parameter("message", "Hello world!");
      // Create publisher
      publisher_ = this->create_publisher<build_my_string::msg::Message>("topic", 10);
      // Create repeating loop that calls timer_callback() every x ms
      timer_ = this->create_wall_timer(
        50ms, std::bind(&MessagePublisher::timer_callback, this)
      );
    }
  private:
    // Function called on repeat by timer_
    void timer_callback()
    {
      // Get paramter
      std::string message = this->get_parameter("message").as_string();
      // Set message to be published
      auto user_msg = build_my_string::msg::Message();
      user_msg.message = message;
      // Publish message
      RCLCPP_INFO(this->get_logger(), "Publishing: %s. [%zu]", user_msg.message.c_str(), this->count_);
      // Increase current count
      this->count_++;
    }
    rclcpp::Publisher<build_my_string::msg::Message>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    // todo Variables to keep track of build progress
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessagePublisher>());
  rclcpp::shutdown();
  return 0;
}


