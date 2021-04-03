#include "triton_controls/waypoint_marker.hpp"
using std::placeholders::_1;

namespace triton_controls
{

  WaypointMarker::WaypointMarker(const rclcpp::NodeOptions & options)
  : Node("waypoint_marker", options),
    num_waypoints_ (0),
    waypoint_values_ (max_waypoints_, std::vector<double>(6, 0.0)), // seems to be in order... might resize in param callback
    has_pose_ (false),
    threshold_type_ ("passby"),
    threshold_dist_ (-1.0),
    threshold_ang_diff_ (-1.0),
    threshold_consect_ (-1),
    threshold_counter_ (0)
  {

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&WaypointMarker::parameter_callback, this, _1)
    );


    this->declare_parameter("num_waypoints", num_waypoints_);

    for (int i = 0; i < max_waypoints_; i++) {
        std::string name = "waypoint_";
        name = name.append(std::to_string(i));

        this->declare_parameter(name + ".x", waypoint_values_[i][0]);
        this->declare_parameter(name + ".y", waypoint_values_[i][1]);
        this->declare_parameter(name + ".z", waypoint_values_[i][2]);
        this->declare_parameter(name + ".roll", waypoint_values_[i][3]);
        this->declare_parameter(name + ".pitch", waypoint_values_[i][4]);
        this->declare_parameter(name + ".yaw", waypoint_values_[i][5]);
    }

    this->declare_parameter("threshold.type", threshold_type_); //should thresholds be per pipeline or per waypoint?
    this->declare_parameter("threshold.distance", threshold_dist_);
    this->declare_parameter("threshold.angle_diff", threshold_ang_diff_);
    this->declare_parameter("threshold.consecutive", threshold_consect_);

    feedback_pub_ = this->create_publisher<triton_interfaces::msg::PipelineFeedback>(
        "/triton/pipeline_feedback", 10
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/triton/controls/input_pose", 10
    );

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/triton/state", 10, std::bind(&WaypointMarker::callback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "New waypoint at: x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f", 
                waypoint_values_.back()[0], waypoint_values_.back()[1], waypoint_values_.back()[2], waypoint_values_.back()[3], waypoint_values_.back()[4], waypoint_values_.back()[5]);
    RCLCPP_INFO(this->get_logger(), "Waypoint Marker successfully started!");
  }


  rcl_interfaces::msg::SetParametersResult WaypointMarker::parameter_callback(const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
        RCLCPP_INFO(this->get_logger(), "Param %s: %s", param.get_name().c_str(), param.value_to_string().c_str());

        if (param.get_name() == "num_waypoints") this->get_parameter(param.get_name(), num_waypoints_);
        
        if (param.get_name().substr(0, 9) == "waypoint_") {
            std::size_t us = param.get_name().find("_");
            std::size_t pt = param.get_name().find(".");
            int i = std::stoi(param.get_name().substr(us+1, pt-us), nullptr, 10);
            
            if (param.get_name().substr(pt, 6) == ".x") this->get_parameter(param.get_name(), waypoint_values_[i][0]);
            if (param.get_name().substr(pt, 6) == ".y") this->get_parameter(param.get_name(), waypoint_values_[i][1]);
            if (param.get_name().substr(pt, 6) == ".z") this->get_parameter(param.get_name(), waypoint_values_[i][2]);
            if (param.get_name().substr(pt, 6) == ".roll") this->get_parameter(param.get_name(), waypoint_values_[i][3]);
            if (param.get_name().substr(pt, 6) == ".pitch") this->get_parameter(param.get_name(), waypoint_values_[i][4]);
            if (param.get_name().substr(pt, 6) == ".yaw") this->get_parameter(param.get_name(), waypoint_values_[i][5]);
        }
            
        if (param.get_name() == "threshold.type") this->get_parameter(param.get_name(), threshold_type_);
        if (param.get_name() == "threshold.distance") this->get_parameter(param.get_name(), threshold_dist_);
        if (param.get_name() == "threshold.angle_diff")  this->get_parameter(param.get_name(), threshold_ang_diff_);
        if (param.get_name() == "threshold.consecutive") this->get_parameter(param.get_name(), threshold_consect_);
    }
    return result;
  }


  void WaypointMarker::callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (!has_pose_) {
      has_pose_ = true;
      pose_ = *msg;
    }

    if (num_waypoints_ == 0) return;

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

            if (distance < threshold_dist_) {
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

            if (distance < threshold_dist_ && angle_diff < threshold_ang_diff_) {
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

            if (distance < threshold_dist_ && angle_diff < threshold_ang_diff_)
                threshold_counter_++;
            else
                threshold_counter_ = 0;

            if (threshold_counter_ >= threshold_consect_) {
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

