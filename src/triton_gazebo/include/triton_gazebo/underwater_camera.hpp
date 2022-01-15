#ifndef TRITON_GAZEBO__UNDERWATER_CAMERA
#define TRITON_GAZEBO__UNDERWATER_CAMERA

#include <memory>
#include <eigen3/Eigen/Core>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "triton_interfaces/msg/detection_box_array.hpp"

typedef sensor_msgs::msg::Image Image;
typedef sensor_msgs::msg::Image::ConstSharedPtr ImageMsg;
typedef message_filters::sync_policies::ApproximateTime<Image, Image> ApproxPolicy;
typedef message_filters::Synchronizer<ApproxPolicy> ApproxSync;


namespace triton_gazebo
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
         * @param bbox_msg bounding box array message (TODO: Make this optional?)
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

        //From 400 to 700 nm in intervals of 25 nm
        typedef Eigen::Array<float,13,1> Array13f;

        //Physical parameters
        Array13f rho_;
        Array13f Beta_;
        Array13f S_b_;
        Array13f S_g_;
        Array13f S_r_;
        Array13f E_0_;

        //Randomized parameters when node is initialized
        float B_b_;
        float B_g_;
        float B_r_;
        float d_;
        
        //Precomputed values when parameters are loaded
        float log_trapz_num_bz_;
        float T_bd_;
        float log_trapz_num_gz_;
        float T_gd_;
        float log_trapz_num_rz_;
        float T_rd_;

        //trapezoidal integration of evenly spaced vector with dx=1
        float trapz(Array13f & vec){
            Eigen::ArrayXf traps = (vec.tail(vec.size()-1) + vec.head(vec.size()-1))/2;
            return traps.sum();
        };
        //Simpson's rule integration of evenly spaced vector with dx=1
        float simps(Array13f & vec){
            float sum = 0;
            int size = vec.size();
            sum = vec(1) + vec(size-1);
            for (int i = 0; i < (size-2)/2; i++){
                sum += 4*vec(1+2*i);
                sum += 2*vec(2+2*i);
            }
            if (size & 1) {//odd
                sum += 4*vec(size-2);
            }
            return sum/3;
        };
        
    };
    
} // namespace triton_gazebo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_gazebo::UnderwaterCamera)

#endif  //TRITON_GAZEBO__UNDERWATER_CAMERA
