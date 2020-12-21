#include "triton_state_maintainer/component_one.hpp"
using std::placeholders::_1;

namespace triton_state_maintainer
{

  ComponentOne::ComponentOne(const rclcpp::NodeOptions & options)
  : Node("component_one", options),
    has_pose_ (false)
  {
    //for (int i = 0; i < 7; i++) {
    //  this->declare_parameter<geometry_msgs::msg::Pose>("pose_offset"+i, pose_offset_value_[i]);
    //  this->get_parameter("pose_offset"+i, pose_offset_value_[i]);
    //}

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/triton/controls/input_pose", 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/triton/state", 10, std::bind(&ComponentOne::callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Started component!");
    RCLCPP_INFO(this->get_logger(), "pose_ x:%f, y:%f, z:%f,  x:%f, y:%f, z:%f, w:%f", pose_.position.x, pose_.position.y, pose_.position.z, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
  }


  void ComponentOne::callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "In Component One, %d", has_pose_);
    if (!has_pose_) {
      has_pose_ = true;
      pose_ = *msg;
    }
    auto message = geometry_msgs::msg::Pose();
    message = pose_;
    message.position.x += pose_offset_.position.x;
    message.position.y += pose_offset_.position.y;
    message.position.z += pose_offset_.position.z;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "thanks!! %f, %f, %f", message.position.x, message.position.y, message.position.z);
  }


} // namespace triton_state_maintainer


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<triton_state_maintainer::ComponentOne>(options));
  rclcpp::shutdown();
  return 0;
}

