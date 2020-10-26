#ifndef TRITON_GAZEBO__UNDERWATER_CAMERA
#define TRITON_GAZEBO__UNDERWATER_CAMERA

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"


namespace gazebo_nodes
{      

    class UnderwaterCamera : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * Create subscriber to gazebo and publishers to publish
         * gazebo images using a QoS agreeable with rQt image view.
         * 
         * @param options ros2 node options
         */
        explicit UnderwaterCamera(const rclcpp::NodeOptions & options);


    private:


        /** Gazebo image callback
         * 
         * @param msg image message 
         */
        void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);


        /** Gazebo depth image callback
         * 
         * @param msg image message 
         */
        void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);


        image_transport::Publisher underwater_img_pub_;
        image_transport::Publisher img_pub_;
        image_transport::Publisher depth_pub_;

        image_transport::Subscriber img_sub_;
        image_transport::Subscriber depth_sub_;
    };
    
} // namespace gazebo_nodes

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gazebo_nodes::UnderwaterCamera)

#endif  //TRITON_GAZEBO__UNDERWATER_CAMERA
