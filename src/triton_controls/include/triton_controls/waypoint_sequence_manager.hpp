#ifndef TRITON_CONTROL__WAYPOINT_SEQUENCE_MANAGER
#define TRITON_CONTROL__WAYPOINT_SEQUENCE_MANAGER

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Matrix3x3.h"
#include "triton_interfaces/msg/pipeline_feedback.hpp"

namespace triton_controls
{

    class WaypointSequenceManager : public rclcpp::Node
    {

    public:

        /** Constructor
         *
         * Creates the waypoint from the given parameters, then sets up pubs/subs
         *
         * @param options ros2 node options.
         */
        explicit WaypointSequenceManager(const rclcpp::NodeOptions & options);

        /** Explanation of intention of wp_type
        * passthrough shall pass through the waypoint with the specific pose
        * stabilize shall come to a stop at the waypoint with the specific pose
        *   wiki wants it to have some time in the threshold?
        *   I thought we previously said that would be something controls would handle
        *   I assumed it was a black box where you told it to stabilize and the robot would do it
        *   and the only remaining work is to check if it has arrived properly, if not, the
        *   controls would have still been active and trying to correct itself?
        */
        /*enum {
            passthrough, orient, stabilize
        } wp_type;*/

    private:

         /** Pose message callback
         *
         * Publishes the waypoint position calculated by taking the waypoint
         * and adding the initial position given in the first message recieved.
         *
         * @param msg geometry pose message witth position and orientation
         */
        void subscriberCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

        rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> & parameters);


        rclcpp::Publisher<triton_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;
        rclcpp::Subscription<triton_interfaces::msg::Success>::SharedPtr subscription_;
        rclcpp::Client<triton_interfaces::srv::MarkWaypoint>::SharedPtr client_;


        const int max_waypoints_ = 20;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        int num_waypoints_;

        std::vector<std::vector<double>> waypoint_values_;
        std::vector<std::string> waypoint_type_values_;
        std::vector<std::vector<double>> waypoint_thresh_values_;

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::WaypointSequenceManager)

#endif  //TRITON_CONTROL__WAYPOINT_SEQUENCE_MANAGER
