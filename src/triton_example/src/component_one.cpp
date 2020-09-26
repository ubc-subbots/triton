#include "triton_example/component_one.hpp"
using std::placeholders::_1;

namespace example
{


ComponentOne::ComponentOne(const rclcpp::NodeOptions & options)
: Node("component_one", options) 
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("component_one/out", 10);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "component_one/in", 10, std::bind(&ComponentOne::callback, this, _1));
}


void ComponentOne::callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "In Component One");
    auto message = std_msgs::msg::String();
    message.data = msg->data + " from ComponentOne";
    publisher_->publish(message);
}

    
} // namespace example