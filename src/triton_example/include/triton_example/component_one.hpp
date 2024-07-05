#ifndef TRITON_EXAMPLE__COMPONENT_ONE
#define TRITON_EXAMPLE__COMPONENT_ONE

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace triton_example
{      

    class ComponentOne : public rclcpp::Node
    {

    public:

        /** Brief description of function.
         * 
         * Longer description in which you describe more in depth about
         * the way the function performs the task mentioned in the brief
         * description above.
         * 
         * @pre precondition of this method, if any
         * 
         * @param options ros2 node options.
         * @param other_param another param
         * 
         * @returns what this function returns, if anything
         * 
         * @post postcondition of this method, if any
         * 
         * @note something of particular note about this function
         * 
         */
        explicit ComponentOne(const rclcpp::NodeOptions & options);

    private:

        /** Brief description of function.
         * 
         * Longer description in which you describe more in depth about
         * the way the function performs the task mentioned in the brief
         * description above.
         * 
         * @pre precondition of this method, if any
         * 
         * @param options ros2 node options.
         * @param other_param another param
         * 
         * @returns what this function returns, if anything
         * 
         * @post postcondition of this method, if any
         * 
         * @note something of particular note about this function
         * 
         */
        void callback(const std_msgs::msg::String::SharedPtr msg) const;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; 

    };
    
} // namespace triton_example

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_example::ComponentOne)

#endif  //TRITON_EXAMPLE__COMPONENT_ONE
