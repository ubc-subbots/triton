#include "triton_example/component_two.hpp"
using std::placeholders::_1;

namespace example
{


ComponentTwo::ComponentTwo(const rclcpp::NodeOptions & options)
: Node("component_two", options) 
{
    feedback_pub_ = this->create_publisher<triton_interfaces::msg::PipelineFeedback>(
        "/triton/pipeline_feedback", 10
    );

    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "component_two/out", 10
    );

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "component_two/in", 10, std::bind(&ComponentTwo::callback, this, _1)
      );

    counter_ = 0;
}


void ComponentTwo::callback(const std_msgs::msg::String::SharedPtr msg) 
{
    RCLCPP_INFO(this->get_logger(), "In Component Two");
    auto message = std_msgs::msg::String();
    message.data = msg->data + " and ComponentTwo";
    publisher_->publish(message);

    counter_++;
    if (counter_ == MESSAGE_THRESHOLD)
    {
        auto feedback_msg = triton_interfaces::msg::PipelineFeedback();
        feedback_msg.success = true;
        feedback_msg.message = "Reached 25! The example pipeline has completed it's action";
        feedback_pub_->publish(feedback_msg);
    }
}

    
} // namespace example