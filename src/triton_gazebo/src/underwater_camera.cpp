#include "triton_gazebo/underwater_camera.hpp"
using std::placeholders::_1;

namespace gazebo_nodes
{

    UnderwaterCamera::UnderwaterCamera(const rclcpp::NodeOptions & options)
    : Node("underwater_camera", options) 
    {
        image_buf_.set_capacity(buf_size_);
        depth_buf_.set_capacity(buf_size_);

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
            std::bind(&UnderwaterCamera::imgCallback, this, _1),
            "raw",
            subscriber_qos_profile
        );

        depth_sub_ = image_transport::create_subscription(this, 
            "/triton/gazebo_drivers/front_camera/depth/image_raw",
            std::bind(&UnderwaterCamera::depthCallback, this, _1),
            "raw",
            subscriber_qos_profile
        );

        RCLCPP_INFO(this->get_logger(), "Underwater Camera succesfully started!");
    }


    void UnderwaterCamera::imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) 
    {
        img_pub_.publish(msg);
        image_buf_.push_back(msg);
        std::pair<ImageMsg, ImageMsg> pair = findClosestPair(); 
    }


    void UnderwaterCamera::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) 
    {
        depth_pub_.publish(msg);
        depth_buf_.push_back(msg);
    }


    std::pair<ImageMsg, ImageMsg> UnderwaterCamera::findClosestPair()
    {
        std::pair<ImageMsg, ImageMsg> pair;
        // TODO
        return pair;
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