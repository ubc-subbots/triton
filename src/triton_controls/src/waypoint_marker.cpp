#include "triton_controls/waypoint_marker.hpp"
using std::placeholders::_1;

namespace triton_controls
{

  WaypointMarker::WaypointMarker(const rclcpp::NodeOptions & options)
  : Node("waypoint_marker", options),
    pose_offset_value_ (6, 0),
    has_pose_ (false)
  {
    this->declare_parameter("pos.x", pose_offset_value_[0]);
    this->declare_parameter("pos.y", pose_offset_value_[1]);
    this->declare_parameter("pos.z", pose_offset_value_[2]);
    this->declare_parameter("ang.r", pose_offset_value_[3]);
    this->declare_parameter("ang.p", pose_offset_value_[4]);
    this->declare_parameter("ang.y", pose_offset_value_[5]);

    this->get_parameter("pos.x", pose_offset_value_[0]);
    this->get_parameter("pos.y", pose_offset_value_[1]);
    this->get_parameter("pos.z", pose_offset_value_[2]);
    this->get_parameter("ang.r", pose_offset_value_[3]);
    this->get_parameter("ang.p", pose_offset_value_[4]);
    this->get_parameter("ang.y", pose_offset_value_[5]);

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/triton/controls/input_pose", 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/triton/state", 10, std::bind(&WaypointMarker::callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Waypoint at: x:%f, y:%f, z:%f,  roll:%f, pitch:%f, yaw:%f", pose_offset_value_[0], pose_offset_value_[1], pose_offset_value_[2], pose_offset_value_[3], pose_offset_value_[4], pose_offset_value_[5], pose_offset_value_[6]);
    RCLCPP_INFO(this->get_logger(), "Waypoint Marker successfully started!");
  }


  void WaypointMarker::callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (!has_pose_) {
      has_pose_ = true;
      pose_ = *msg;
    }
    auto reply_msg = geometry_msgs::msg::Pose();
    reply_msg = pose_;
    reply_msg.position.x += pose_offset_value_[0];
    reply_msg.position.y += pose_offset_value_[1];
    reply_msg.position.z += pose_offset_value_[2];

    tf2::Quaternion quat;
    quat.setRPY(pose_offset_value_[3], pose_offset_value_[4], pose_offset_value_[5]);

    reply_msg.orientation.x = quat.x();
    reply_msg.orientation.y = quat.y();
    reply_msg.orientation.z = quat.z();
    reply_msg.orientation.w = quat.w();

    publisher_->publish(reply_msg);

  }


} // namespace triton_controls

