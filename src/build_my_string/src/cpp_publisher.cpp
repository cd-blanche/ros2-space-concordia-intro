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
    MessagePublisher() :
      Node("message_publisher"),
      count_(0),
      base_width(20),
      base_height(40)
    {
      // Initialize custom parameters
      this->declare_parameter("message", "Hello world!");
      this->declare_parameter("base_w", base_width);
      this->declare_parameter("base_h", base_height);

      cached_base_width = base_width;
      cached_base_height = base_height;
      cached_str = "Hello world!";

      // Initialize base layer
      set_base_w();

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
      // Get parameters
      std::string message = this->get_parameter("message").as_string();
      this->get_parameter("base_w", base_width);
      this->get_parameter("base_h", base_height);

      // Check to see if param still matches cached string
      if (message != cached_str) {
        cached_str = message;
        count_ = 0; // Reset build count
      }
      if (base_width != cached_base_width) {
        cached_base_width = base_width;
        count_ = 0;
        base_str = "";
        set_base_w();
      }
      if (base_height != cached_base_height) {
        cached_base_height = base_height;
        count_ = 0;
      }

      // Print output
      print_output();

      // Increase current count
      this->count_++;
    }
    
    // Publisher variables
    rclcpp::Publisher<build_my_string::msg::Message>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // todo Add variables to keep track of build progress
    size_t count_;

    int base_width;
    int base_height;
    int cached_base_width;
    int cached_base_height;
    std::string base_str;
    std::string cached_str;

    // todo Function to build output
    void print_output()
    {
      // Print base layer
      print_base();

      // Print player
      print_player();

      // Set message to be published
      // auto user_msg = build_my_string::msg::Message();
      // user_msg.message = message;

      // Publish message
      // RCLCPP_INFO(this->get_logger(), "[cache: %s] Publishing: %s. [%zu]", cached_str.c_str(), user_msg.message.c_str(), this->count_);
    }

    // Function to print player layer
    void print_player()
    {
      std::string player_str = base_str;
      player_str[0] = '(';
      player_str[1] = '>';
      player_str[2] = 'o';
      player_str[3] = '_';
      player_str[4] = 'o';
      player_str[5] = ')';
      player_str[6] = '>';


      RCLCPP_INFO(this->get_logger(), player_str.c_str());
    }

    // Function to print base layer
    void print_base()
    {
      for (int i = 0; i < base_height; i++)
      {
        RCLCPP_INFO(this->get_logger(), base_str.c_str());
      }
    }

    // Function to set base width
    void set_base_w()
    {
      for (int i = 0; i < base_width; i++)
      {
        base_str += ".";
      }
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessagePublisher>());
  rclcpp::shutdown();
  return 0;
}


