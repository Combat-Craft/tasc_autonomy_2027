#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    //optional: too specify description and constraints
    /*
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";
    this->declare_parameter("my_parameter", "world", param_desc);
    // note the 3rd argument is the parameter 
    */

    // THIS IS A PARAMETER LOOK HERE!!!!
    // its NAME is "my_parameter", and its DEFAULT VALUE is "world"
    // type is inferred from the default value, so here its a string
    this->declare_parameter("my_parameter", "world");
    // NOTE: parameters can get changed at runtime, so you can change them via the command line or a launch file, and the node will pick up the new value on the next timer callback
    
    auto timer_callback = [this](){
      // note that the type must be decalred, and match the default value!
      // this is how we actually retreive and USE the paramters
      std::string my_param = this->get_parameter("my_parameter").as_string();

      RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

      // we want to store all of our parameters
      std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
      this->set_parameters(all_new_parameters);
    };
    timer_ = this->create_wall_timer(1000ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}