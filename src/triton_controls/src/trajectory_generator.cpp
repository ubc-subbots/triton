#include "triton_controls/trajectory_generator.hpp"
using std::placeholders::_1;

namespace triton_controls
{

  TrajectoryGenerator::TrajectoryGenerator(const rclcpp::NodeOptions &options)
      : Node("trajectory_generator", options),
        type_(0),
        destination_achieved_(false)
  {

    waypoint_publisher_ = this->create_publisher<triton_interfaces::msg::Waypoint>("/triton/controls/waypoint_marker/set", 10);

    state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/triton/controls/ukf/odometry/filtered", 10, std::bind(&TrajectoryGenerator::state_callback, this, _1));

    type_subscription_ = this->create_subscription<triton_interfaces::msg::TrajectoryType>(
        "/triton/controls/trajectory_generator/set_type", 10, std::bind(&TrajectoryGenerator::type_callback, this, _1));

    gate_subscription_ = this->create_subscription<triton_interfaces::msg::ObjectOffset>(
        "/triton/gate/detector/gate_pose", 10, std::bind(&TrajectoryGenerator::gate_callback, this, _1));

    waypoint_subscription_ = this->create_subscription<triton_interfaces::msg::Waypoint>(
        "/triton/controls/waypoint_marker/current_goal", 10, std::bind(&TrajectoryGenerator::waypoint_callback, this, _1));

    // this->declare_parameter("start_turning_factor", start_turning_factor_);
    // this->get_parameter("start_turning_factor", start_turning_factor_);

    RCLCPP_INFO(this->get_logger(), "Trajectory Generator successfully started!");
  }

  void TrajectoryGenerator::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    current_pose_ = msg->pose.pose;
    if (type_ == TRAJ_START) 
    {
      // Turn the AUV around slowly (to search for gate)
      auto reply_msg = triton_interfaces::msg::Waypoint();
      reply_msg.pose = msg->pose.pose;

      tf2::Quaternion current_pose_q(
        current_pose_.orientation.x,
        current_pose_.orientation.y,
        current_pose_.orientation.z,
        current_pose_.orientation.w);
      tf2::Matrix3x3 current_pose_q_m(current_pose_q);
      double current_pose_roll, current_pose_pitch, current_pose_yaw;
      current_pose_q_m.getRPY(current_pose_roll, current_pose_pitch, current_pose_yaw);

      // Set some small yaw offset
      tf2::Quaternion tf2_quat_dest;
      tf2_quat_dest.setRPY(0.001, 0.001, current_pose_yaw -1.57); // todo: add back roll and pitch if we control them in the future
      reply_msg.pose.orientation.x = tf2_quat_dest.x();
      reply_msg.pose.orientation.y = tf2_quat_dest.y();
      reply_msg.pose.orientation.z = tf2_quat_dest.z();
      reply_msg.pose.orientation.w = tf2_quat_dest.w();

      // Set some small distance
      tf2::Quaternion tf2_quat_distance;
      tf2_quat_distance.setRPY(0.05, 0.05, 0.1);

      reply_msg.distance.position.x = 0.2;
      reply_msg.distance.position.y = 0.2;
      reply_msg.distance.position.z = 0.2;
      reply_msg.distance.orientation.x = tf2_quat_distance.x();
      reply_msg.distance.orientation.y = tf2_quat_distance.y();
      reply_msg.distance.orientation.z = tf2_quat_distance.z();
      reply_msg.distance.orientation.w = tf2_quat_distance.w();
      reply_msg.duration = 2;
      reply_msg.type = 0; // STABILIZE

      waypoint_publisher_->publish(reply_msg);
    }
    else if (type_ == TRAJ_GATE)
    {
      // TOOD: generate trajectory
      if (!destination_achieved_)
      {

        auto reply_msg = triton_interfaces::msg::Waypoint();

        // reply_msg is sent to the Waypoint marker. It is in the map frame. 
        // destination_pose_ is from the gate detector. It is in the base frame. 
        // current_pose_ is also in the map frame. 

        // Forward and sidway components (assuming AUV is upright, no change in z)
        tf2::Quaternion current_q;
        tf2::Vector3 dest_v;
        dest_v.setX(destination_pose_.position.x);
        dest_v.setY(destination_pose_.position.y);
        dest_v.setZ(0);
        tf2::fromMsg(current_pose_.orientation, current_q); 
        tf2::Vector3 targetForward = tf2::quatRotate(current_q, dest_v);
        reply_msg.pose.position.x = current_pose_.position.x + targetForward.getX();
        reply_msg.pose.position.y = current_pose_.position.y + targetForward.getY();

        // Z
        reply_msg.pose.position.z = current_pose_.position.z + destination_pose_.position.z;


        // If the gate is not in front (on the left or right in the image), 
        // turn towards it. 
        tf2::Quaternion current_pose_q(
          current_pose_.orientation.x,
          current_pose_.orientation.y,
          current_pose_.orientation.z,
          current_pose_.orientation.w);
        tf2::Matrix3x3 current_pose_q_m(current_pose_q);
        double current_pose_roll, current_pose_pitch, current_pose_yaw;
        current_pose_q_m.getRPY(current_pose_roll, current_pose_pitch, current_pose_yaw);
        // TODO: optimize, check math
        // destination_pose_.position.y is in meters, should be in the single digits
        double required_yaw = std::max(-1.57, std::min(1.57, destination_pose_.position.y/1.0 * 1.57));

        tf2::Quaternion tf2_quat_destination;
        tf2_quat_destination.setRPY(current_pose_roll, current_pose_pitch, current_pose_yaw + required_yaw + 0.2); // 0.2 is a magic number for triton_auv in gazebo
        reply_msg.pose.orientation.x = tf2_quat_destination.x();
        reply_msg.pose.orientation.y = tf2_quat_destination.y();
        reply_msg.pose.orientation.z = tf2_quat_destination.z();
        reply_msg.pose.orientation.w = tf2_quat_destination.w();

        // Set some small distance
        tf2::Quaternion tf2_quat_distance;
        tf2_quat_distance.setRPY(0.05, 0.05, 0.05); 

        reply_msg.distance.position.x = 0.2;
        reply_msg.distance.position.y = 0.2;
        reply_msg.distance.position.z = 0.2;
        reply_msg.distance.orientation.x = tf2_quat_distance.x();
        reply_msg.distance.orientation.y = tf2_quat_distance.y();
        reply_msg.distance.orientation.z = tf2_quat_distance.z();
        reply_msg.distance.orientation.w = tf2_quat_distance.w();
        reply_msg.type = 1;  // PASSTHROUGH

        waypoint_publisher_->publish(reply_msg);
      }
    }

  }

  void TrajectoryGenerator::type_callback(const triton_interfaces::msg::TrajectoryType::SharedPtr msg)
  {

    type_ = msg->type;
    destination_achieved_ = false;

    RCLCPP_INFO(this->get_logger(), "Trajectory Type updated. ");

  }

  void TrajectoryGenerator::gate_callback(const triton_interfaces::msg::ObjectOffset::SharedPtr msg)
  {

    if (msg->class_id == type_ && type_ == TRAJ_GATE)
    {
      // Catch extreme cases when the gate detector did not detect properly
      if (abs(msg->pose.position.x) < 50 
          && abs(msg->pose.position.y) < 50
          && abs(msg->pose.position.z) < 20)
      {
        destination_achieved_ = false;
        destination_pose_ = msg->pose;
      }
    }

  }

  void TrajectoryGenerator::waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg)
  {

    if (msg->success)
    {
      destination_achieved_ = true;
    }

  }

} // namespace triton_controls

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<triton_controls::TrajectoryGenerator>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const&){} // during testing sometimes throws error
  return 0;
}

