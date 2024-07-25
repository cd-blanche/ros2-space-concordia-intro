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
      // Cache string
      cached_str = "Hello world!";
      // Create publisher
      publisher_ = this->create_publisher<build_my_string::msg::Message>("topic", 10);
      // Create repeating loop that calls timer_callback() every x ms
      timer_ = this->create_wall_timer(
        500ms, std::bind(&MessagePublisher::timer_callback, this)
      );
    }
  private:
    // Function called on repeat by timer_
    void timer_callback()
    {
      // Get parameter
      std::string message = this->get_parameter("message").as_string();

      // Check to see if param still matches cached string
      if (message != cached_str) {
        cached_str = message;
        count_ = 0; // Reset build count
      }

      // Print output
      print_output(message);

      // Increase current count
      this->count_++;
    }
    
    // Publisher variables
    rclcpp::Publisher<build_my_string::msg::Message>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // todo Add variables to keep track of build progress
    size_t count_;
    std::string cached_str;
    std::string base_str;

    // todo Function to build output
    void print_output(const std::string message)
    {
      // Set message to be published
      auto user_msg = build_my_string::msg::Message();
      user_msg.message = message;
      // Publish message
      RCLCPP_INFO(this->get_logger(), "[cache: %s] Publishing: %s. [%zu]", cached_str.c_str(), user_msg.message.c_str(), this->count_);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessagePublisher>());
  rclcpp::shutdown();
  return 0;
}


