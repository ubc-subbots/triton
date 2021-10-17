#include "triton_controls/waypoint_sequence_manager.hpp"
using std::placeholders::_1;

namespace triton_controls
{
  WaypointSequenceManager::WaypointSequenceManager(const rclcpp::NodeOptions & options)
  : Node("waypoint_sequence_manager", options),
    num_waypoints_ (0),
    waypoint_pose_values_ (max_waypoints_, std::vector<double>(6, 0.0)),
    waypoint_type_values_ (max_waypoints_, "stabilize"),
    waypoint_thresh_values_ (max_waypoints_, std::vector<double>(2, 0.0))
  {

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&WaypointSequenceManager::parameter_callback, this, _1)
    );


    this->declare_parameter("num_waypoints", num_waypoints_);

    for (int i = 0; i < max_waypoints_; i++) {
        std::string name = "waypoint_";
        name = name.append(std::to_string(i));

        this->declare_parameter(name + "pose.x", waypoint_values_[i][0]);
        this->declare_parameter(name + "pose.y", waypoint_values_[i][1]);
        this->declare_parameter(name + "pose.z", waypoint_values_[i][2]);
        this->declare_parameter(name + "pose.roll", waypoint_values_[i][3]);
        this->declare_parameter(name + "pose.pitch", waypoint_values_[i][4]);
        this->declare_parameter(name + "pose.yaw", waypoint_values_[i][5]);

        this->declare_parameter(name + "type", waypoint_type_values_[i]); //string to enum?

        this->declare_parameter(name + "threshold.position", waypoint_thresh_values_[i][0]);
        this->declare_parameter(name + "threshold.rotation", waypoint_thresh_values_[i][1]);
    }

    feedback_pub_ = this->create_publisher<triton_interfaces::msg::PipelineFeedback>(
        "/triton/pipeline_feedback", 10
    );

    subscription_ = this->create_subscription<triton_interfaces::msg::Success>(
      "/triton/waypoint/feedback",
      10,
      std::bind(&WaypointSequenceManager::subscriberCallback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "New waypoint at: x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f",
                waypoint_values_.back()[0], waypoint_values_.back()[1], waypoint_values_.back()[2], waypoint_values_.back()[3], waypoint_values_.back()[4], waypoint_values_.back()[5]);
    RCLCPP_INFO(this->get_logger(), "Waypoint Sequence Manager successfully started!");

    // put the service here? Following from the tutorial.
    // set quaternion something like this:
    //To be continued
  client_ = this->create_client<triton_interfaces::srv::MarkWaypoint>("/triton/waypoint/waypoint_service");

  // I can't just start sending requests here since parameters might not be getted?
    //If complete we can send the pipeline success
  }


  rcl_interfaces::msg::SetParametersResult WaypointSequenceManager::parameter_callback(const std::vector<rclcpp::Parameter> & parameters)
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

            if (param.get_name().substr(pt, 6) == ".pose.x") this->get_parameter(param.get_name(), waypoint_values_[i][0]);
            if (param.get_name().substr(pt, 6) == ".pose.y") this->get_parameter(param.get_name(), waypoint_values_[i][1]);
            if (param.get_name().substr(pt, 6) == ".pose.z") this->get_parameter(param.get_name(), waypoint_values_[i][2]);
            if (param.get_name().substr(pt, 6) == ".pose.roll") this->get_parameter(param.get_name(), waypoint_values_[i][3]);
            if (param.get_name().substr(pt, 6) == ".pose.pitch") this->get_parameter(param.get_name(), waypoint_values_[i][4]);
            if (param.get_name().substr(pt, 6) == ".pose.yaw") this->get_parameter(param.get_name(), waypoint_values_[i][5]);

            if (param.get_name().substr(pt, 6) == ".threshold.type") this->get_parameter(param.get_name(), waypoint_type_values_[i]);

            if (param.get_name().substr(pt, 6) == ".threshold.position") this->get_parameter(param.get_name(), waypoint_thresh_values_[i][0]);
            if (param.get_name().substr(pt, 6) == ".threshold.rotation") this->get_parameter(param.get_name(), waypoint_thresh_values_[i][1]);
        }
    }
    //call service to first waypoint, if it exists
    // call again in the subscriber callback
    if (num_waypoints_ > 0) {
      auto request = std::make_shared<triton_interfaces::srv::MarkWaypoint::Request>();
      request->pose.position.x = waypoint_values_[num_waypoints - 1][0];
      request->pose.position.y = waypoint_values_[num_waypoints - 1][1];
      request->pose.position.z = waypoint_values_[num_waypoints - 1][2];

      tf2::Quaternion quat;
      quat.setRPY(waypoint_values_[num_waypoints - 1][3], waypoint_values_[num_waypoints - 1][4], waypoint_values_[num_waypoints - 1][5]);

      request->pose.orientation.x = quat.x;
      request->pose.orientation.y = quat.y;
      request->pose.orientation.z = quat.z;
      request->pose.orientation.w = quat.w;

      request->type = waypoint_values_[num_waypoints - 1];
      request->displacement_tolerance = waypoint_thresh_values_[0];
      request->angular_tolerance = waypoint_thresh_values_[1];

      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return result;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto response = client_->async_send_request(request);
      // Wait for the response.
      if (rclcpp::spin_until_future_complete(node, response) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully called waypoint_service");
        num_waypoints_--;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service waypoint_service");
      }
    }
    return result;
  }


  void WaypointMarker::subscriberCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (msg->success) {
      if (num_waypoints_ > 0) {
        auto request = std::make_shared<triton_interfaces::srv::MarkWaypoint::Request>();
        request->pose.position.x = waypoint_values_[num_waypoints - 1][0];
        request->pose.position.y = waypoint_values_[num_waypoints - 1][1];
        request->pose.position.z = waypoint_values_[num_waypoints - 1][2];

        tf2::Quaternion quat;
        quat.setRPY(waypoint_values_[num_waypoints - 1][3], waypoint_values_[num_waypoints - 1][4], waypoint_values_[num_waypoints - 1][5]);

        request->pose.orientation.x = quat.x;
        request->pose.orientation.y = quat.y;
        request->pose.orientation.z = quat.z;
        request->pose.orientation.w = quat.w;

        request->type = waypoint_values_[num_waypoints - 1];
        request->displacement_tolerance = waypoint_thresh_values_[0];
        request->angular_tolerance = waypoint_thresh_values_[1];

        while (!client_->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return result;
          }
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto response = client_->async_send_request(request);
        // Wait for the response.
        if (rclcpp::spin_until_future_complete(node, response) == rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully called waypoint_service");
          num_waypoints_--;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service waypoint_service");
        }
      }
    }
  }


} // namespace triton_controls

