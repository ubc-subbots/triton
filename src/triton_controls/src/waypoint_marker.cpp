#include "triton_controls/waypoint_marker.hpp"
using std::placeholders::_1;

namespace triton_controls
{

  WaypointMarker::WaypointMarker(const rclcpp::NodeOptions & options)
  : Node("waypoint_marker", options),
    //pose_offset_value_ (6, 0),
    num_waypoints_ (1),
    waypoint_values_ (1, std::vector<double>(6, 0)),
    has_pose_ (false),
    threshold_counter_ (0)
  {
    /*this->declare_parameter("pos.x", pose_offset_value_[0]);
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
    this->get_parameter("ang.y", pose_offset_value_[5]);*/

    this->declare_parameter("num_waypoints", num_waypoints_);
    this->get_parameter("num_waypoints", num_waypoints_);

    waypoint_values_.resize(num_waypoints_, std::vector<double>(6, 0));

    // The waypoints are stored in the vector from back to front.
    for (int i = 0; i < num_waypoints_; i++) {
        std::string name = "waypoint_";
        name = name.append(std::to_string(i));

        uint32_t j = num_waypoints_ - i - 1;

        this->declare_parameter(name.append(".x"), waypoint_values_[j][0]);
        this->declare_parameter(name.append(".y"), waypoint_values_[j][1]);
        this->declare_parameter(name.append(".z"), waypoint_values_[j][2]);
        this->declare_parameter(name.append(".roll"), waypoint_values_[j][3]);
        this->declare_parameter(name.append(".pitch"), waypoint_values_[j][4]);
        this->declare_parameter(name.append(".yaw"), waypoint_values_[j][5]);

        this->get_parameter(name.append(".x"), waypoint_values_[j][0]);
        this->get_parameter(name.append(".y"), waypoint_values_[j][1]);
        this->get_parameter(name.append(".z"), waypoint_values_[j][2]);
        this->get_parameter(name.append(".roll"), waypoint_values_[j][3]);
        this->get_parameter(name.append(".pitch"), waypoint_values_[j][4]);
        this->get_parameter(name.append(".yaw"), waypoint_values_[j][5]);
    }

    this->declare_parameter("threshold.type", threshold_type_); //should thresholds be per pipeline or per waypoint?
    this->get_parameter("threshold.type", threshold_type_);

    if (threshold_type_ == "passby") {
        this->declare_parameter("threshold.distance", threshold_value_0_);
        this->get_parameter("threshold.distance", threshold_value_0_);
    } else if (threshold_type_ == "orient") {
        this->declare_parameter("threshold.distance", threshold_value_0_);
        this->declare_parameter("threshold.ang_diff", threshold_value_1_);
        this->get_parameter("threshold.distance", threshold_value_0_);
        this->get_parameter("threshold.ang_diff", threshold_value_1_);
    } else if (threshold_type_ == "stabilize") {
        this->declare_parameter("threshold.distance", threshold_value_0_);
        this->declare_parameter("threshold.ang_diff", threshold_value_1_);
        this->declare_parameter("threshold.consecutive", threshold_value_2_);
        this->get_parameter("threshold.distance", threshold_value_0_);
        this->get_parameter("threshold.ang_diff", threshold_value_1_);
        this->get_parameter("threshold.consecutive", threshold_value_2_);
    }

    feedback_pub_ = this->create_publisher<triton_interfaces::msg::PipelineFeedback>(
        "/triton/pipeline_feedback", 10
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/triton/controls/input_pose", 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/triton/state", 10, std::bind(&WaypointMarker::callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "New waypoint at: x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f", 
                waypoint_values_.back()[0], waypoint_values_.back()[1], waypoint_values_.back()[2], waypoint_values_.back()[3], waypoint_values_.back()[4], waypoint_values_.back()[5]);
    RCLCPP_INFO(this->get_logger(), "Waypoint Marker successfully started!");
  }


  void WaypointMarker::callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (!has_pose_) {
      has_pose_ = true;
      pose_ = *msg;
    }

    auto reply_msg = geometry_msgs::msg::Pose();
    bool trigger_thresh;
    bool abort = false;
    std::string abort_msg;

    do {
        trigger_thresh = false;

        reply_msg = pose_;

        tf2::Quaternion quat;
        quat.setRPY(waypoint_values_.back()[3], waypoint_values_.back()[4], waypoint_values_.back()[5]);

        reply_msg.position.x += waypoint_values_.back()[0];
        reply_msg.position.y += waypoint_values_.back()[1];
        reply_msg.position.z += waypoint_values_.back()[2];

        reply_msg.orientation.x = quat.x();
        reply_msg.orientation.y = quat.y();
        reply_msg.orientation.z = quat.z();
        reply_msg.orientation.w = quat.w();

        if (threshold_type_ == "passby") {
            // Passby will only check the waypoint position, useful if you are passing by.

            //Euclidean distance by Pythogoreas
            double distance = std::pow(msg->position.x - reply_msg.position.x, 2) + std::pow(msg->position.y - reply_msg.position.y, 2) + std::pow(msg->position.z - reply_msg.position.z, 2);
            distance = std::sqrt(distance);

            if (distance < threshold_value_0_) {
                trigger_thresh = true;
                waypoint_values_.pop_back();
            }

        } else if (threshold_type_ == "orient") {
            // Orient will also check the angular distance to the waypoint orientation, if you want to point somewhere.

            //Euclidean distance by Pythogoreas
            double distance = std::pow(msg->position.x - reply_msg.position.x, 2) + std::pow(msg->position.y - reply_msg.position.y, 2) + std::pow(msg->position.z - reply_msg.position.z, 2);
            distance = std::sqrt(distance);

            //Angular difference by comparing the quaternions
            double angle_diff = 2 * quat.angle(tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));

            if (distance < threshold_value_0_ && angle_diff < threshold_value_1_) {
                trigger_thresh = true;
                waypoint_values_.pop_back();
            }

        } else if (threshold_type_ == "stabilize") {
            // Stabilize is like orient, but it will wait for a certain number of messages

            //Euclidean distance by Pythogoreas
            double distance = std::pow(msg->position.x - reply_msg.position.x, 2) + std::pow(msg->position.y - reply_msg.position.y, 2) + std::pow(msg->position.z - reply_msg.position.z, 2);
            distance = std::sqrt(distance);

            //Angular difference by comparing the quaternions
            double angle_diff = quat.angle(tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));

            if (distance < threshold_value_0_ && angle_diff < threshold_value_1_)
                threshold_counter_++;
            else
                threshold_counter_ = 0;

            if (threshold_counter_ >= threshold_value_2_) {
                threshold_counter_ = 0;
                trigger_thresh = true;
                waypoint_values_.pop_back();
            }

        } else {
            abort = true;
            abort_msg = "Waypoint aborted, unrecognized threshold type...";
        }

        if (trigger_thresh) {
            if (waypoint_values_.size() == 0) {
                auto feedback_msg = triton_interfaces::msg::PipelineFeedback();
                feedback_msg.success = true;
                feedback_msg.message = "No more waypoints! The waypoint pipeline has completed it's action";
                feedback_pub_->publish(feedback_msg);
            } else {
                RCLCPP_INFO(this->get_logger(), "New waypoint at: x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f",
                            waypoint_values_.back()[0], waypoint_values_.back()[1], waypoint_values_.back()[2], waypoint_values_.back()[3], waypoint_values_.back()[4], waypoint_values_.back()[5]);
            }
        }
        if (abort) {
            auto feedback_msg = triton_interfaces::msg::PipelineFeedback();
            feedback_msg.abort = true;
            feedback_msg.message = abort_msg;
            feedback_pub_->publish(feedback_msg);
        }
    } while (trigger_thresh);

    publisher_->publish(reply_msg);

  }


} // namespace triton_controls

