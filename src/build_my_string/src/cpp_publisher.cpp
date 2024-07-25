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
      base_height(40),
      playerPos(0),
      playerDirection(RIGHT)
    {
      // Initialize custom parameters
      this->declare_parameter("message", "Hello world!");
      this->declare_parameter("base_w", base_width);
      this->declare_parameter("base_h", base_height);

      cached_base_width = base_width;
      cached_base_height = base_height;
      cached_inp_msg = "Hello world!";

      // Initialize base layer
      set_base_width();

      // Create publisher
      publisher_ = this->create_publisher<build_my_string::msg::Message>("topic", 10);

      // Create repeating loop that calls timer_callback() every x ms
      timer_ = this->create_wall_timer(
        100ms, std::bind(&MessagePublisher::timer_callback, this)
      );
    }

  private:
    // Function called on repeat by timer_
    void timer_callback()
    {
      // Get parameters
      this->get_parameter("message", input_message);
      this->get_parameter("base_w", base_width);
      this->get_parameter("base_h", base_height);

      // Check to see if param still matches cached string
      if (input_message != cached_inp_msg) {
        cached_inp_msg = input_message;
        count_ = 0; // Reset build count
      }
      if (base_width != cached_base_width) {
        cached_base_width = base_width;
        count_ = 0;
        base_layer = "";
        set_base_width();
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
    enum PlayerDirection {
      LEFT,
      RIGHT
    };

    size_t count_;

    // base layer
    int base_width;
    int base_height;
    std::string base_layer;
    int cached_base_width;
    int cached_base_height;

    // player layer
    int playerPos;
    PlayerDirection playerDirection;

    // message input
    std::string input_message;
    std::string cached_inp_msg;

    // todo Function to build output
    void print_output()
    {
      // Print base layer
      print_base();

      // Print player
      print_player();

      // Set message to be published
      auto user_msg = build_my_string::msg::Message();
      user_msg.message = input_message;

      // Publish message
      RCLCPP_INFO(this->get_logger(), "[cache: %s] Publishing: %s. [%zu]", cached_inp_msg.c_str(), user_msg.message.c_str(), this->count_);
    }

    // Function to print player layer
    void print_player()
    {
      std::string player_layer = base_layer;

      // Increment based on direction player is moving
      if (playerDirection == RIGHT) {
        playerPos++;
        player_layer[playerPos] = '(';
        player_layer[playerPos + 1] = '>';
        player_layer[playerPos + 2] = 'o';
        player_layer[playerPos + 3] = '_';
        player_layer[playerPos + 4] = 'o';
        player_layer[playerPos + 5] = ')';
        player_layer[playerPos + 6] = '>';
        // player_layer[playerPos + 7] = 'a';
      } else if (playerDirection == LEFT) {
        playerPos--;
        player_layer[playerPos] = '<';
        player_layer[playerPos + 1] = '(';
        player_layer[playerPos + 2] = 'o';
        player_layer[playerPos + 3] = '_';
        player_layer[playerPos + 4] = 'o';
        player_layer[playerPos + 5] = '<';
        player_layer[playerPos + 6] = ')';
      }

      // Once player reaches far end of the base, reverse direction
      if (playerPos == base_width - 7) {
        playerDirection = LEFT;
      } else if (playerPos == 0) {
        playerDirection = RIGHT;
      }

      RCLCPP_INFO(this->get_logger(), player_layer.c_str());
    }

    // Function to print base layer
    void print_base()
    {
      for (int i = 0; i < base_height; i++)
      {
        RCLCPP_INFO(this->get_logger(), base_layer.c_str());
      }
    }

    // Function to set base width
    void set_base_width()
    {
      for (int i = 0; i < base_width; i++)
      {
        base_layer += ".";
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


