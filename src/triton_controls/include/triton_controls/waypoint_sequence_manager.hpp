#ifndef TRITON_CONTROL__WAYPOINT_SEQUENCE_MANAGER
#define TRITON_CONTROL__WAYPOINT_SEQUENCE_MANAGER

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
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

    private:

         /** Pose message callback
         *
         * Publishes the waypoint position calculated by taking the waypoint
         * and adding the initial position given in the first message recieved.
         *
         * @param msg geometry pose message witth position and orientation
         */
        rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> & parameters);

        rclcpp::Publisher<triton_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;

        const int max_waypoints_ = 20;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        bool has_pose_;
        geometry_msgs::msg::Pose pose_;

        /** Explanation of intention of wp_type
        * passby shall pass through the waypoint without stopping, ignoring rotation
        * orient shall pass through the waypoint without stopping, while rotating
        * stabilize shall stop at the waypoint then rotate
        */
        enum {
            passby, orient, stabilize
        } wp_type;

        int num_waypoints_;

        std::vector<std::vector<double>> waypoint_pose_values_;
        std::vector<wp_type> waypoint_type_values_;
        std::vector<std::vector<double>> waypoint_thresh_values_;

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::WaypointSequenceManager)

#endif  //TRITON_CONTROL__WAYPOINT_SEQUENCE_MANAGER
