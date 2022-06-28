#ifndef TRITON_CONTROL__SERIAL_SUBSCRIBER 
#include "std_msgs/msg/u_int32.hpp"

#include "rclcpp/rclcpp.hpp"
namespace triton_controls
{      

    class SerialSubscriber : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * @param options ros2 node options.
         */
        explicit SerialSubscriber(const rclcpp::NodeOptions & options);
        ~SerialSubscriber();

    private:
        void controlCallback(const std_msgs::msg::UInt32::SharedPtr msg) const;
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr thruster_sub_; 
        int fd_;
    };
    
} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::SerialSubscriber)

#endif  //TRITON_CONTROL__SERIAL_SUBSCRIBER
