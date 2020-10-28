#include "triton_gazebo/underwater_camera.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


namespace gazebo_nodes
{

    UnderwaterCamera::UnderwaterCamera(const rclcpp::NodeOptions & options)
    : Node("underwater_camera", options) 
    {

        rmw_qos_profile_t subscriber_qos_profile = rmw_qos_profile_sensor_data;
        rmw_qos_profile_t publisher_qos_profile = rmw_qos_profile_default;

        underwater_image_pub_ = image_transport::create_publisher(this, 
            "image_raw", 
            publisher_qos_profile);

        image_pub_ = image_transport::create_publisher(this, 
            "gazebo/image_raw", 
            publisher_qos_profile);

        depth_pub_ = image_transport::create_publisher(this, 
            "gazebo/depth/image_raw", 
            publisher_qos_profile);

        image_sub_.subscribe(this,
            "/triton/gazebo_drivers/front_camera/image_raw", 
            "raw",
            subscriber_qos_profile);

        depth_sub_.subscribe(this,
             "/triton/gazebo_drivers/front_camera/depth/image_raw", 
             "raw",
             subscriber_qos_profile);

        approx_sync_ = std::make_shared<ApproxSync>(
            ApproxPolicy(5),
            image_sub_, 
            depth_sub_);

        approx_sync_->registerCallback(
            std::bind(&UnderwaterCamera::syncCallback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Underwater Camera succesfully started!");
    }


    void UnderwaterCamera::syncCallback(const ImageMsg & image_msg, const ImageMsg & depth_msg)
    {   
        float image_stamp = image_msg->header.stamp.sec+image_msg->header.stamp.nanosec*1e-9;
        float depth_stamp = depth_msg->header.stamp.sec+depth_msg->header.stamp.nanosec*1e-9;
        float time_diff = std::abs(image_stamp-depth_stamp);
        if (time_diff > 0)
        {
            // Show non zero time difference between image pairs, never seems to be above 0.07s
            RCLCPP_INFO(this->get_logger(), "Non-Zero Time Difference: [%.5f]", time_diff);
        }
        image_pub_.publish(image_msg);
        depth_pub_.publish(depth_msg);
        underwaterImageSynthesis(image_msg, depth_msg);
    }


    void UnderwaterCamera::underwaterImageSynthesis(const ImageMsg & image_msg, const ImageMsg & depth_msg)
    {
        auto message = sensor_msgs::msg::Image();
        // TODO: implement underwater synthesis algorithm
        RCLCPP_WARN(this->get_logger(), "Underwater synthesis algorithm not implemented!");
        underwater_image_pub_.publish(message);
    }


    
} // namespace gazebo_nodes


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<gazebo_nodes::UnderwaterCamera>(options));
  rclcpp::shutdown();
  return 0;
}