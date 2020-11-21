#include "triton_state_maintainer/component_one.hpp"
using std::placeholders::_1;

namespace triton_state_maintainer
{


    ComponentOne::ComponentOne(const rclcpp::NodeOptions & options)
    : Node("component_one", options),
        has_pose_ (false)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/triton/controls/input_pose", 10);

        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
          "/triton/state", 10, std::bind(&ComponentOne::callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Started component!");
    }


    void ComponentOne::callback(const geometry_msgs::msg::Pose::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "In Component One, %d", has_pose_);
        if (!has_pose_) {
            has_pose_ = true;
            pose_ = *msg;
        }
        auto message = geometry_msgs::msg::Pose();
        message = pose_;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "hi!! %f", message.position.x);
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
