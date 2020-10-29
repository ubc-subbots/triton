#ifndef TRITON_CONTROL__THRUST_ALLOCATOR
#define TRITON_CONTROL__THRUST_ALLOCATOR

#include <math.h>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace triton_controls
{      

    class ThrustAllocator : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * TODO: Better explanation
         * 
         * @param options ros2 node options.
         */
        explicit ThrustAllocator(const rclcpp::NodeOptions & options);

    private:

        /** Wrench message callback
         * 
         * TODO: Better explanation
         * 
         * @param msg wrench message with 3D force/torque to allocates
         */
        void wrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg) const;

        /** Cosine and sine in degrees
         * 
         * @param deg angle in degrees
         */
        inline float cosd(float deg) { return cos( deg*M_PI/180.0); }
        inline float sind(float deg) { return cos( deg*M_PI/180.0); }

        /** Helper for getting magnitude/direction of thrust from x/y parts
         * 
         * @param x the x-component of the thrust
         * @param y the y-component of the thrust
         */
        inline float getThrust(float x, float y) const{ 
            return copysign(1.0,x)*sqrt(pow(x,2) + pow(y,2));
        }

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_pub_;  
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr signals_pub_;  

        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr forces_sub_; 

        cv::Mat alloc;


    };
    
} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::ThrustAllocator)

#endif  //TRITON_CONTROL__THRUST_ALLOCATOR
