#include "triton_controls/thrust_allocator.hpp"
using std::placeholders::_1;

namespace triton_controls
{


ThrustAllocator::ThrustAllocator(const rclcpp::NodeOptions & options)
: Node("thrust_allocator", options) 
{
    forces_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "output_forces", 10);

    signals_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "signals", 10);

    forces_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "/triton/controls/input_forces", 10, std::bind(&ThrustAllocator::wrenchCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Thrust Allocator succesfully started!");
}


void ThrustAllocator::wrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) const
{

}
    
} // namespace triton_controls


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<triton_controls::ThrustAllocator>(options));
  rclcpp::shutdown();
  return 0;
}
