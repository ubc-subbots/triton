#ifndef TRITON_CONTROL__WAYPOINT_MARKER
#define TRITON_CONTROL__WAYPOINT_MARKER

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "triton_interfaces/msg/waypoint.hpp"

namespace triton_controls
{

    class WaypointMarker : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * Creates the waypoint from the given parameters, then sets up pubs/subs
         * 
         * @param options ros2 node options.
         */
        explicit WaypointMarker(const rclcpp::NodeOptions & options);

    private:

         /** State message callback
         * 
         * Updates private variable containing the AUV's current pose. 
         * Determines whether the destination pose is reached if one is set. 
         * 
         * @param msg geometry pose message with position and orientation
         */
        void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

         /** Waypoint message callback
         * 
         * Updates destination waypoint. 
         * 
         * @param msg triton_interfaces waypoint message with pose and waypoint type
         */
        void waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg);

        rclcpp::Publisher<triton_interfaces::msg::Waypoint>::SharedPtr publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;
        rclcpp::Subscription<triton_interfaces::msg::Waypoint>::SharedPtr waypoint_subscription_;

        bool waypoint_set_;                         // Whether a waypoint is set
        geometry_msgs::msg::Pose waypoint_pose_;    // Destination waypoint pose
        geometry_msgs::msg::Pose current_pose_;     // AUV current pose
        geometry_msgs::msg::Pose max_pose_offset_;  // Distance criterion
        float min_stabilize_duration_;              // Time criterion
        rclcpp::Time last_stable_start_time_;       // Time stamp of the last time meeting waypoint criteria
        bool waypoint_achieved_;                    // Whether a waypoint is achieved

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::WaypointMarker)

#endif  //TRITON_CONTROL__WAYPOINT_MARKER
