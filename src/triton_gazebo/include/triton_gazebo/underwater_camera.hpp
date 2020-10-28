#ifndef TRITON_GAZEBO__UNDERWATER_CAMERA
#define TRITON_GAZEBO__UNDERWATER_CAMERA

#include <memory>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

typedef sensor_msgs::msg::Image Image;
typedef sensor_msgs::msg::Image::ConstSharedPtr ImageMsg;
typedef message_filters::sync_policies::ApproximateTime<Image, Image> ApproxPolicy;
typedef message_filters::Synchronizer<ApproxPolicy> ApproxSync;

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


        /** Synced image/depth message callback
         * 
         * Called whenever a pair of image/depth message have approximatley
         * equal timestamps. Publishes each image through an rqt image view compatible 
         * publisher then performs the underwater algorithm on the pair and publishes 
         * the result
         * 
         * @param image_msg image message 
         * @param depth_msg depth message 
         */
        void syncCallback(const ImageMsg & image_msg, const ImageMsg & depth_msg);


        /** Performs underwater synthesis on a image/depth pair, publishes result
         * 
         * TODO: Further description of the algorithm
         * 
         * @param image_msg image message 
         * @param depth_msg depth message 
         */
        void underwaterImageSynthesis(const ImageMsg & image_msg, const ImageMsg & depth_msg);


        image_transport::Publisher underwater_image_pub_;
        image_transport::Publisher image_pub_;
        image_transport::Publisher depth_pub_;

        image_transport::SubscriberFilter image_sub_;
        image_transport::SubscriberFilter depth_sub_;  

        std::shared_ptr<ApproxSync> approx_sync_; 

    };
    
} // namespace gazebo_nodes

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gazebo_nodes::UnderwaterCamera)

#endif  //TRITON_GAZEBO__UNDERWATER_CAMERA
