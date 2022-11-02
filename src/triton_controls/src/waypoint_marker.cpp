#include "triton_controls/waypoint_marker.hpp"
using std::placeholders::_1;

namespace triton_controls
{

  WaypointMarker::WaypointMarker(const rclcpp::NodeOptions & options)
  : Node("waypoint_marker", options),
    waypoint_set_ (false),
    waypoint_achieved_ (false)
  {

    publisher_ = this->create_publisher<triton_interfaces::msg::Waypoint>("/triton/controls/input_pose", 10);

    state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/triton/controls/ukf/odometry/filtered", 10, std::bind(&WaypointMarker::state_callback, this, _1));

    waypoint_subscription_ = this->create_subscription<triton_interfaces::msg::Waypoint>(
      "/triton/controls/waypoint_marker/set", 10, std::bind(&WaypointMarker::waypoint_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Waypoint Marker successfully started!");
  }


  void WaypointMarker::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    current_pose_ = msg->pose.pose;

    if (waypoint_set_)
    {
      auto reply_msg = triton_interfaces::msg::Waypoint();
      reply_msg.pose = current_pose_;

      publisher_->publish(reply_msg);

    }

  }


  void WaypointMarker::waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg)
  {
    auto reply_msg = triton_interfaces::msg::Waypoint();
    reply_msg.pose = current_pose_;

    publisher_->publish(reply_msg);

  }
} // namespace triton_controls

