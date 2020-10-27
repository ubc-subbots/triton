#ifndef TRITON_GAZEBO__UNDERWATER_CAMERA
#define TRITON_GAZEBO__UNDERWATER_CAMERA

#include <utility>

#include <boost/circular_buffer.hpp>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

typedef sensor_msgs::msg::Image::ConstSharedPtr ImageMsg;

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
         * Adds the message to the image buffer and publishes it. Also
         * find the closest depth/image message pair, performs the
         * underwater algorithm on the pair and publishes the result.
         * 
         * @param msg image message 
         */
        void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);


        /** Gazebo depth image callback
         * 
         * Adds the message to the depth buffer and publishes it
         * 
         * @param msg image message 
         */
        void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);


        /** Finds the closest temporal pair of depth/image message pairs
         * 
         * If both buffers are full, given the middle image message in the 
         * image buffer, finds the depth message in the depth buffer which is 
         * closest temporally and returns the pair of messages.
         * 
         * @returns pair of image message, first being raw image, second being depth
         */
        std::pair<ImageMsg, ImageMsg> findClosestPair();


        image_transport::Publisher underwater_img_pub_;
        image_transport::Publisher img_pub_;
        image_transport::Publisher depth_pub_;

        image_transport::Subscriber img_sub_;
        image_transport::Subscriber depth_sub_;   

        int buf_size_ = 3; // MUST be an odd integer
        boost::circular_buffer<ImageMsg> image_buf_;
        boost::circular_buffer<ImageMsg> depth_buf_;

    };
    
} // namespace gazebo_nodes

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gazebo_nodes::UnderwaterCamera)

#endif  //TRITON_GAZEBO__UNDERWATER_CAMERA
