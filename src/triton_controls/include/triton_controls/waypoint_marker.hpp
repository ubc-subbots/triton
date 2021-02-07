#ifndef TRITON_CONTROL__WAYPOINT_MARKER
#define TRITON_CONTROL__WAYPOINT_MARKER

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace triton_controls
{

    class WaypointMarker : public rclcpp::Node
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
        /** Constructor
         * 
         * Creates the allocation matrix from the given parameters, then stores the 
         * pseudoinverse of that matrix and sets up pubs/subs
         * 
         * @param options ros2 node options.
         */
        explicit WaypointMarker(const rclcpp::NodeOptions & options);

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
         /** Wrench message callback
         * 
         * Calculates the thrust by multiplying the pseudo inverse of the allocation
         * matrix by the vector of forces/torques given by wrench and publishing the
         * result to the output forces topic
         * 
         * thrust = pinv(A)*tau
         * 
         * @param msg wrench message with 3D force/torque to allocates
         */
        void callback(const geometry_msgs::msg::Pose::SharedPtr msg);

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;

        bool has_pose_;
        geometry_msgs::msg::Pose pose_;
        std::vector<double> pose_offset_value_;
        geometry_msgs::msg::Pose pose_offset_;

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::WaypointMarker)

#endif  //TRITON_CONTROL__WAYPOINT_MARKER
