// these 3 are dependencies required for pushblisher ros2 c++ nodes
// they have to be added in package.xml and CMakelists.txt!
#include <chrono>    // publsihing only
#include <memory>    // for all nodes
#include <string>    // cause we're using a string msg type
// I suggest you comment why/where you need other includes

#include "rclcpp/rclcpp.hpp"       // convience header needed for nodes
#include "std_msgs/msg/string.hpp" //this is the publisher msg type

// if you add more header files, you must place them in the include folder 
// and add them to the CMakeLists.txt file, otherwise the compiler will not find them

using namespace std::chrono_literals; // default

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

// https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
// please read this doc more a more flesh out explanation
class MinimalPublisher : public rclcpp::Node // inherent from Node class
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) // node name is "minimal_publisher"
  {
    // string msg type publisher, topic name is "topic", queue size is 10
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        // init the msg 
        auto message = std_msgs::msg::String(); 

        // set the msg data
        message.data = "Hello, world! " + std::to_string(this->count_++); 

        // og to console using a macro
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); 

        // the actual publishing step. should see it via ros2 topic echo /topic
        this->publisher_->publish(message); 
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

// declares the publisher, and node parameter fields
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

// the actual node executes here. Normally you don't edit anything here
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}