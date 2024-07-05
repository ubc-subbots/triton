#ifndef TRITON_CONTROL__WAYPOINT_MARKER
#define TRITON_CONTROL__WAYPOINT_MARKER

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include "triton_interfaces/msg/waypoint.hpp"
#include <math.h>

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

        // Current waypoint status
        rclcpp::Publisher<triton_interfaces::msg::Waypoint>::SharedPtr publisher_;
        // Goal minus current state. For the PID Controller
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr error_publisher_;
        // Current state of AUV
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;
        // New goal
        rclcpp::Subscription<triton_interfaces::msg::Waypoint>::SharedPtr waypoint_subscription_;

        bool waypoint_set_;                         // Whether a waypoint is set
        triton_interfaces::msg::Waypoint waypoint_; // Destination waypoint pose
        double waypoint_roll, waypoint_pitch, waypoint_yaw; // Waypoint orientation in RPY
        double distance_roll, distance_pitch, distance_yaw; // Waypoint distance orientation in RPY
        geometry_msgs::msg::Pose current_pose_;     // AUV current pose
        rclcpp::Time last_stable_start_time_;       // Time stamp of the last time meeting waypoint criteria
        bool waypoint_achieved_;                    // Whether a waypoint is achieved
        bool waypoint_being_achieved_;              // Whether the AUV is within 'distance' to the destination
        geometry_msgs::msg::Pose error_pose_;       // Error of goal minus current pose

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::WaypointMarker)

#endif  //TRITON_CONTROL__WAYPOINT_MARKER
