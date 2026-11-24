// unlike the publisher, only need memory and rclcpp for basic subscriber nodes
#include <memory>
// I suggest you comment why/where you need such includes

#include "rclcpp/rclcpp.hpp"      // convinience header needed for nodes
#include "std_msgs/msg/string.hpp" //this is the publisher msg type

// if you add more header files, you must place them in the include folder 
// and add them to the CMakeLists.txt file, otherwise the compiler will not find them

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  //like publisher, node name is "minimal_subscriber", but no count
  : Node("minimal_subscriber")  
  // no timer! bc it responds to msgs, not periodically publishes them
  {
    // the callback, which reacts to a msg being published
    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        // logs to console to indicate the subscriber works and heard
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    // note that the topic name is the same as the publisher, and the queue size is also 10
    // THEY MUST MATCH, OTHERWISE THE SUBSCRIBER WILL NOT RECEIVE ANY MESSAGES
    subscription_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
  }

// defaukt only needs the subscription field, but can more as needed
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}