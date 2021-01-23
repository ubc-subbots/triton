#ifndef TRITON_STATE_MAINTAINER__COMPONENT_ONE
#define TRITON_STATE_MAINTAINER__COMPONENT_ONE

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace triton_state_maintainer
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
        void callback(const geometry_msgs::msg::Pose::SharedPtr msg);

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
        
        bool has_pose_;
        geometry_msgs::msg::Pose pose_;
        std::vector<double> pose_offset_value_; // = std::vector<double>(7, 100.0);
        geometry_msgs::msg::Pose pose_offset_;

    };

} // namespace triton_example

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_state_maintainer::ComponentOne)

#endif  //TRITON_STATE_MAINTAINER__COMPONENT_ONE
