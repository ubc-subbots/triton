#include "triton_gazebo/underwater_camera.hpp"
using std::placeholders::_1;

namespace gazebo_nodes
{

    UnderwaterCamera::UnderwaterCamera(const rclcpp::NodeOptions & options)
    : Node("underwater_camera", options) 
    {
        rmw_qos_profile_t subscriber_qos_profile = rmw_qos_profile_sensor_data;
        rmw_qos_profile_t publisher_qos_profile = rmw_qos_profile_default;

        underwater_img_pub_ = image_transport::create_publisher(this, 
            "image_raw", publisher_qos_profile);

        img_pub_ = image_transport::create_publisher(this, 
            "gazebo/image_raw", publisher_qos_profile);

        depth_pub_ = image_transport::create_publisher(this, 
            "gazebo/depth/image_raw", publisher_qos_profile);

        img_sub_ = image_transport::create_subscription(this, 
            "/triton/gazebo_drivers/front_camera/image_raw",
            std::bind(&UnderwaterCamera::img_callback, this, _1),
            "raw",
            subscriber_qos_profile
        );

        depth_sub_ = image_transport::create_subscription(this, 
            "/triton/gazebo_drivers/front_camera/depth/image_raw",
            std::bind(&UnderwaterCamera::depth_callback, this, _1),
            "raw",
            subscriber_qos_profile
        );

        RCLCPP_INFO(this->get_logger(), "Underwater Camera succesfully started!");
    }


    void UnderwaterCamera::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) 
    {
        float time = msg->header.stamp.sec +  msg->header.stamp.nanosec*1e-9;
        RCLCPP_INFO(this->get_logger(), "Image Raw Time: [%f]", time);
        img_pub_.publish(msg);
    }


    void UnderwaterCamera::depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) 
    {
        float time = msg->header.stamp.sec +  msg->header.stamp.nanosec*1e-9;
        RCLCPP_INFO(this->get_logger(), "Depth Image Time: [%f]", time);
        depth_pub_.publish(msg);
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