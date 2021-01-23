#include "triton_state_maintainer/component_one.hpp"
using std::placeholders::_1;

namespace triton_state_maintainer
{

  ComponentOne::ComponentOne(const rclcpp::NodeOptions & options)
  : Node("component_one", options),
    has_pose_ (false)
  {
    //this->declare_parameter<std::vector<double>>("pose_offset", pose_offset_value_);
    //this->get_parameter("pose_offset", pose_offset_value_);

    this->declare_parameter("pose_offset.pos.x", pose_offset_value_[0]);
    this->declare_parameter("pose_offset.pos.y", pose_offset_value_[1]);
    this->declare_parameter("pose_offset.pos.z", pose_offset_value_[2]);
    this->declare_parameter("pose_offset.orient.x", pose_offset_value_[3]);
    this->declare_parameter("pose_offset.orient.y", pose_offset_value_[4]);
    this->declare_parameter("pose_offset.orient.z", pose_offset_value_[5]);
    this->declare_parameter("pose_offset.orient.w", pose_offset_value_[6]);

    this->get_parameter("pose_offset.pos.x", pose_offset_value_[0]);
    this->get_parameter("pose_offset.pos.y", pose_offset_value_[1]);
    this->get_parameter("pose_offset.pos.z", pose_offset_value_[2]);
    this->get_parameter("pose_offset.orient.x", pose_offset_value_[3]);
    this->get_parameter("pose_offset.orient.y", pose_offset_value_[4]);
    this->get_parameter("pose_offset.orient.z", pose_offset_value_[5]);
    this->get_parameter("pose_offset.orient.w", pose_offset_value_[6]);

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/triton/controls/input_pose", 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/triton/state", 10, std::bind(&ComponentOne::callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Started component!");
    RCLCPP_INFO(this->get_logger(), "pose_ x:%f, y:%f, z:%f,  x:%f, y:%f, z:%f, w:%f", pose_.position.x, pose_.position.y, pose_.position.z, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
    RCLCPP_INFO(this->get_logger(), "pose_offset_value_ size = %d", pose_offset_value_.size());
    //RCLCPP_INFO(this->get_logger(), "pose_offset_value_ x:%f, y:%f, z:%f,  x:%f, y:%f, z:%f, w:%f", pose_offset_value_[0], pose_offset_value_[1], pose_offset_value_[2], pose_offset_value_[3], pose_offset_value_[4], pose_offset_value_[5], pose_offset_value_[6]);
  }


  void ComponentOne::callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "In Component One, %d", has_pose_);
    if (!has_pose_) {
      has_pose_ = true;
      pose_ = *msg;
    }
    auto reply_msg = geometry_msgs::msg::Pose();
    reply_msg = pose_;
    reply_msg.position.x += pose_offset_value_[0]; //pose_offset_.position.x;
    reply_msg.position.y += pose_offset_value_[1]; //pose_offset_.position.y;
    reply_msg.position.z += pose_offset_value_[2]; //pose_offset_.position.z;
    publisher_->publish(reply_msg);
    RCLCPP_INFO(this->get_logger(), "get %f, %f, %f", msg->position.x, msg->position.y, msg->position.z);
    RCLCPP_INFO(this->get_logger(), "reply %f, %f, %f", reply_msg.position.x, reply_msg.position.y, reply_msg.position.z);
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

