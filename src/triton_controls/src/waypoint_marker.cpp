#include "triton_controls/waypoint_marker.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

namespace triton_controls
{

  WaypointMarker::WaypointMarker(const rclcpp::NodeOptions & options)
  : Node("waypoint_marker", options),
    waypoint_pose_values_ (7, 0.0),
    threshold_type_ ("stabilize"),
    waypoint_thresh_values_ ({10.0, 180.0}),
    has_waypoint_ (false),
    threshold_counter_ (0)
  {

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&WaypointMarker::parameter_callback, this, _1)
    );

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
      "/triton/controls/input_pose",
      10
    );

    success_publisher_ = this->create_publisher<triton_interfaces::msg::Success>(
      "/triton/waypoint/feedback",
      10
    );

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/triton/state",
      10,
      std::bind(&WaypointMarker::subscriberCallback, this, _1)
    );

    service_ = this->create_service<triton_interfaces::srv::MarkWaypoint>(
      "/triton/waypoint/waypoint_service",
      std::bind(&WaypointMarker::serviceCallback, this, _1, _2)
    );

    RCLCPP_INFO(this->get_logger(), "Waypoint Marker successfully started!");
  }


  void WaypointMarker::subscriberCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    // Get initial pose offset
//    if (!has_pose_) {
//      has_pose_ = true;
//      pose_ = *msg;
//    }

    auto reply_msg = geometry_msgs::msg::Pose();
    bool trigger_thresh = false;
//    bool abort = false; // do I stil need this
//    std::string abort_msg;

    auto success_msg = triton_interfaces::msg::Success();

    if (!has_waypoint_) {
      // no waypoint obtained yet?
      // do nothing and send nothing?
      return;
    }

//    reply_msg = pose_;

    reply_msg.position.x = waypoint_pose_values_[0];
    reply_msg.position.y = waypoint_pose_values_[1];
    reply_msg.position.z = waypoint_pose_values_[2];
    reply_msg.orientation.x = waypoint_pose_values_[3];
    reply_msg.orientation.y = waypoint_pose_values_[4];
    reply_msg.orientation.z = waypoint_pose_values_[5];
    reply_msg.orientation.w = waypoint_pose_values_[6];

    tf2::Quaternion quat = tf2::Quaternion(waypoint_pose_values_[3], waypoint_pose_values_[4], waypoint_pose_values_[5], waypoint_pose_values_[6]);

    if (threshold_type_ == "passthrough") {
      // Euclidean distance by Pythagoreas
      double distance = std::pow(msg->position.x - reply_msg.position.x, 2) + std::pow(msg->position.y - reply_msg.position.y, 2) + std::pow(msg->position.z - reply_msg.position.z, 2);
      distance = std::sqrt(distance);

      // Angular difference by comparing the quaternions
      double angle_diff = quat.angleShortestPath(tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));

      if (distance < waypoint_thresh_values_[0] && angle_diff < waypoint_thresh_values_[1]) {
        trigger_thresh = true;
      }
    } else if (threshold_type_ == "stabilize") {
      // Euclidean distance by Pythagoreas
      double distance = std::pow(msg->position.x - reply_msg.position.x, 2) + std::pow(msg->position.y - reply_msg.position.y, 2) + std::pow(msg->position.z - reply_msg.position.z, 2);
      distance = std::sqrt(distance);

      // Angular difference by comparing the quaternions
      double angle_diff = quat.angleShortestPath(tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w)); // test this out, see if format works

      if (distance < waypoint_thresh_values_[0] && angle_diff < waypoint_thresh_values_[1]) {
        threshold_counter_++;
        if (threshold_counter_ >= threshold_consect_) {
          trigger_thresh = true;
        } else {
          rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(wait_time_));
        }
      } else {
        threshold_counter_ = 0;
      }
    } else {
        // we have a threshold type I haven't seen before???
        // some sort of feedback to complain
    }

    if (trigger_thresh) {
      success_msg.success = true;
      success_publisher_->publish(success_msg);
      RCLCPP_INFO(this->get_logger(), "New waypoint at: x:%f, y:%f, z:%f; x:%f, y:%f, z:%f, w:%f",
                  waypoint_pose_values_[0], waypoint_pose_values_[1], waypoint_pose_values_[2], waypoint_pose_values_[3], waypoint_pose_values_[4], waypoint_pose_values_[5], waypoint_pose_values_[6]);
    }
  }


  void WaypointMarker::serviceCallback(const triton_interfaces::srv::MarkWaypoint::Request::SharedPtr request,
                                       const triton_interfaces::srv::MarkWaypoint::Response::SharedPtr response)
  {
    waypoint_pose_values_[0] = request->pose.position.x;
    waypoint_pose_values_[1] = request->pose.position.y;
    waypoint_pose_values_[2] = request->pose.position.z;
    waypoint_pose_values_[3] = request->pose.orientation.x;
    waypoint_pose_values_[4] = request->pose.orientation.y;
    waypoint_pose_values_[5] = request->pose.orientation.z;
    waypoint_pose_values_[6] = request->pose.orientation.w;
    threshold_type_ = request->type;
    waypoint_thresh_values_[0] = request->displacement_tolerance;
    waypoint_thresh_values_[1] = request->angular_tolerance;
    //how do I send a response???
    // I need to either wait here for the the state is within threshold or have some other way to send message
    // the plan now is to send success here, and then send a message on a topic once the waypoint is done.
    response->success = true;
  }

} // namespace triton_controls

