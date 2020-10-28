#ifndef TRITON_CONTROL__THRUST_ALLOCATOR
#define TRITON_CONTROL__THRUST_ALLOCATOR

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

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

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr forces_pub_;  
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr signals_pub_;  

        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr forces_sub_; 

    };
    
} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::ThrustAllocator)

#endif  //TRITON_CONTROL__THRUST_ALLOCATOR
