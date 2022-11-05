#include "triton_controls/waypoint_marker.hpp"
using std::placeholders::_1;

namespace triton_controls
{

  WaypointMarker::WaypointMarker(const rclcpp::NodeOptions &options)
      : Node("waypoint_marker", options),
        waypoint_set_(false),
        waypoint_achieved_(false),
        waypoint_being_achieved_(false)
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

      // 1. Determine if current_pose_ is within max_pose_offset_ of waypoint_pose_

      // Calculate the orientation difference, by converting to tf2 Quaternia, 
      // then multiplying one by the inverse of the other
      tf2::Quaternion tf2_quat_from_msg, tf2_quat_waypoint;
      tf2::convert(current_pose_.orientation, tf2_quat_from_msg);
      tf2::convert(waypoint_.pose.orientation, tf2_quat_waypoint);
      // quat_waypoint = quat_diff * quat_msg
      tf2::Quaternion tf2_quat_difference = tf2_quat_waypoint * tf2_quat_from_msg.inverse();

      if (tf2_quat_difference.x() <= waypoint_.distance.orientation.x
        && tf2_quat_difference.y() <= waypoint_.distance.orientation.y
        && tf2_quat_difference.z() <= waypoint_.distance.orientation.z
        && tf2_quat_difference.w() <= waypoint_.distance.orientation.w // TODO: check the math
        && current_pose_.position.x - waypoint_.pose.position.x <= waypoint_.distance.position.x
        && current_pose_.position.y - waypoint_.pose.position.y <= waypoint_.distance.position.y
        && current_pose_.position.z - waypoint_.pose.position.z <= waypoint_.distance.position.z)
      {
        // 2.1. If we are within max_pose_offset_, check duration
        if (waypoint_being_achieved_)
        {
          rclcpp::Time now = this->now();
          if ((now - last_stable_start_time_).seconds() >= waypoint_.duration)
          {
            waypoint_achieved_ = true;
            waypoint_set_ = false; // If waypoint is achieved, then we wait for the next one

            RCLCPP_INFO(this->get_logger(), "Stabilize Waypoint at: (x:%f, y:%f, z:%f), (x:%f, y:%f, z:%f, w:%f) achieved!", 
                        waypoint_.pose.position.x,
                        waypoint_.pose.position.y,
                        waypoint_.pose.position.z,
                        waypoint_.pose.orientation.x,
                        waypoint_.pose.orientation.y,
                        waypoint_.pose.orientation.z,
                        waypoint_.pose.orientation.w
                        );
          }
          // else keep waiting
        }
        else
        {
          if (waypoint_.type == waypoint_.STABILIZE) 
          {
            waypoint_being_achieved_ = true;
            last_stable_start_time_ = this->now();
          }
          else if (waypoint_.type == waypoint_.PASSTHROUGH)
          {
            // Passthrough waypoints have no duration requirements. 
            waypoint_achieved_ = true;
            waypoint_set_ = false; // If waypoint is achieved, then we wait for the next one

            RCLCPP_INFO(this->get_logger(), "Passthrough Waypoint at: (x:%f, y:%f, z:%f), (x:%f, y:%f, z:%f, w:%f) achieved!", 
                        waypoint_.pose.position.x,
                        waypoint_.pose.position.y,
                        waypoint_.pose.position.z,
                        waypoint_.pose.orientation.x,
                        waypoint_.pose.orientation.y,
                        waypoint_.pose.orientation.z,
                        waypoint_.pose.orientation.w
                        );
          }
          else 
          {
            RCLCPP_WARN(this->get_logger(), "Unknown waypoint type. Resetting waypoint marker. ");

            waypoint_set_ = false;
            waypoint_achieved_ = false;
            waypoint_being_achieved_ = false;
            return;

          }
        }
      }
      else
      {
        // 2.2 If we are not within max_pose_offset_, reset variables
        waypoint_being_achieved_ = false;
      }

      auto reply_msg = triton_interfaces::msg::Waypoint();
      reply_msg.pose = waypoint_.pose;
      reply_msg.distance = waypoint_.distance;
      reply_msg.duration = waypoint_.duration;
      reply_msg.success = waypoint_achieved_;
      reply_msg.type = waypoint_.type;

      publisher_->publish(reply_msg);
    }
    // else waypoint is not set, do nothing

  }

  void WaypointMarker::waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg)
  {

    if (waypoint_set_)
    {
      // If it is the same waypoint, nothing is done.

      // Calculate the orientation difference, by converting to tf2 Quaternia, 
      // then multiplying one by the inverse of the other
      tf2::Quaternion tf2_quat_from_msg, tf2_quat_waypoint;
      tf2::convert(msg->pose.orientation, tf2_quat_from_msg);
      tf2::convert(waypoint_.pose.orientation, tf2_quat_waypoint);
      // quat_waypoint = quat_diff * quat_msg_inverse
      tf2::Quaternion tf2_quat_difference = tf2_quat_waypoint * tf2_quat_from_msg.inverse();

      if (tf2_quat_difference.x() == 0 
        && tf2_quat_difference.y() == 0
        && tf2_quat_difference.z() == 0
        && tf2_quat_difference.w() == 0 // TODO: check the math
        && msg->pose.position.x - waypoint_.pose.position.x == 0
        && msg->pose.position.y - waypoint_.pose.position.y == 0
        && msg->pose.position.z - waypoint_.pose.position.z == 0)
      {
        RCLCPP_INFO(this->get_logger(), "Attempt to set an identical waypoint ignored. ");
        return;
      }
    }

    waypoint_set_ = true;
    waypoint_.pose = msg->pose;
    waypoint_.duration = msg->duration;
    waypoint_.distance = msg->distance;
    waypoint_achieved_ = false;
    waypoint_being_achieved_ = false;
    waypoint_.type = msg->type;

    auto reply_msg = triton_interfaces::msg::Waypoint();
    reply_msg.pose = waypoint_.pose;
    reply_msg.success = waypoint_achieved_;
    reply_msg.type = waypoint_.type;
    reply_msg.distance = waypoint_.distance;
    reply_msg.duration = waypoint_.duration;

    publisher_->publish(reply_msg);
  }
} // namespace triton_controls
