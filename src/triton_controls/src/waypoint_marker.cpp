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

    publisher_ = this->create_publisher<triton_interfaces::msg::Waypoint>("/triton/controls/waypoint_marker/current_goal", 10);

    error_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/triton/controls/input_pose", 10);

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

      // Calculate the orientation difference, by converting to RPY
      tf2::Quaternion current_pose_q(
        current_pose_.orientation.x,
        current_pose_.orientation.y,
        current_pose_.orientation.z,
        current_pose_.orientation.w);
      tf2::Matrix3x3 current_pose_q_m(current_pose_q);
      double current_pose_roll, current_pose_pitch, current_pose_yaw;
      current_pose_q_m.getRPY(current_pose_roll, current_pose_pitch, current_pose_yaw);

      double error_roll, error_pitch, error_yaw;
      error_roll = waypoint_roll - current_pose_roll;
      error_pitch = waypoint_pitch - current_pose_pitch;
      error_yaw = waypoint_yaw - current_pose_yaw;

      tf2::Quaternion tf2_quat_difference;
      tf2_quat_difference.setRPY(0.001, 0.001, -error_yaw); // TODO: add back roll and pitch if we control them in the future

      // Assign differences to error_pose_
      // error_pose_ is in the base frame, so we need some rotation
      // TODO: traj generator just converted gate pose to map frame, and now we are converting it back
      tf2::Quaternion current_q;
      tf2::Vector3 error_v;
      error_v.setX(waypoint_.pose.position.x - current_pose_.position.x);
      error_v.setY(waypoint_.pose.position.y - current_pose_.position.y);
      error_v.setZ(waypoint_.pose.position.z - current_pose_.position.z);
      tf2::fromMsg(current_pose_.orientation, current_q); 
      tf2::Vector3 error_v_final = tf2::quatRotate(current_q, error_v);
      error_pose_.position.x = error_v_final.getX();
      error_pose_.position.y = error_v_final.getY();
      error_pose_.position.z = error_v_final.getZ();
      error_pose_.orientation.x = tf2_quat_difference.x();
      error_pose_.orientation.y = tf2_quat_difference.y();
      error_pose_.orientation.z = tf2_quat_difference.z();
      error_pose_.orientation.w = tf2_quat_difference.w();

      if ( fabs(error_roll) <= fabs(distance_roll)
        && fabs(error_pitch) <= fabs(distance_pitch)
        && fabs(error_yaw) <= fabs(distance_yaw)
        && fabs(error_pose_.position.x) <= waypoint_.distance.position.x
        && fabs(error_pose_.position.y) <= waypoint_.distance.position.y
        && fabs(error_pose_.position.z) <= waypoint_.distance.position.z)
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
    else if (waypoint_achieved_) 
    {
      // Publish the last waypoint, so that the AUV stabilizes at current pose
      auto reply_msg = triton_interfaces::msg::Waypoint();
      reply_msg.pose = waypoint_.pose;
      reply_msg.distance = waypoint_.distance;
      reply_msg.duration = waypoint_.duration;
      reply_msg.success = waypoint_achieved_;
      reply_msg.type = waypoint_.type;

      publisher_->publish(reply_msg);

    }
    // else waypoint is not set

    // If a waypoint is set, then error_pose_ contains the error
    // Else, it will be set to all 0s, so the AUV stays still
    if (!waypoint_set_) 
    {
      error_pose_.position.x = 0;
      error_pose_.position.y = 0;
      error_pose_.position.z = 0;
      error_pose_.orientation.x = 0;
      error_pose_.orientation.y = 0;
      error_pose_.orientation.z = 0;
      error_pose_.orientation.w = 0;
    }
    error_publisher_->publish(error_pose_);

  }

  void WaypointMarker::waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg)
  {

    if (waypoint_set_)
    {
      // If it is the same waypoint, nothing is done.

      // Calculate the orientation difference, by converting to tf2 Quaternia, 
      // then multiplying one by the inverse of the other
      tf2::Quaternion tf2_quat_from_msg, tf2_quat_waypoint;
      tf2::fromMsg(msg->pose.orientation, tf2_quat_from_msg);
      tf2::fromMsg(waypoint_.pose.orientation, tf2_quat_waypoint);
      // quat_waypoint = quat_diff * quat_msg_inverse
      tf2::Quaternion tf2_quat_difference = tf2_quat_waypoint * tf2_quat_from_msg.inverse();

      // TODO: set a margin of error. E.g. distances within 1cm are considered 0
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

    // Calculate and store waypoint orientation in RPY
    tf2::Quaternion waypoint_q(
      waypoint_.pose.orientation.x,
      waypoint_.pose.orientation.y,
      waypoint_.pose.orientation.z,
      waypoint_.pose.orientation.w);
    tf2::Matrix3x3 waypoint_q_m(waypoint_q);
    waypoint_q_m.getRPY(waypoint_roll, waypoint_pitch, waypoint_yaw);

    tf2::Quaternion distance_q(
      waypoint_.distance.orientation.x,
      waypoint_.distance.orientation.y,
      waypoint_.distance.orientation.z,
      waypoint_.distance.orientation.w);
    tf2::Matrix3x3 distance_q_m(distance_q);
    distance_q_m.getRPY(distance_roll, distance_pitch, distance_yaw);


    auto reply_msg = triton_interfaces::msg::Waypoint();
    reply_msg.pose = waypoint_.pose;
    reply_msg.success = waypoint_achieved_;
    reply_msg.type = waypoint_.type;
    reply_msg.distance = waypoint_.distance;
    reply_msg.duration = waypoint_.duration;

    // RCLCPP_INFO(this->get_logger(), "A new waypoint is set. ");
    publisher_->publish(reply_msg);
  }
} // namespace triton_controls

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<triton_controls::WaypointMarker>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const&){} // during testing sometimes throws error
  return 0;
}

