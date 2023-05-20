#ifndef TRITON_CONTROL__TRAJECTORY_GENERATOR
#define TRITON_CONTROL__TRAJECTORY_GENERATOR

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include "triton_interfaces/msg/waypoint.hpp"
#include "triton_interfaces/msg/object_offset.hpp"
#include "triton_interfaces/msg/trajectory_type.hpp"
#include <math.h>

// Trajectory type
#define TRAJ_START 0
#define TRAJ_GATE 1

namespace triton_controls
{

    class TrajectoryGenerator : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * Creates the trajectory generator from the given parameters, then sets up pubs/subs
         * 
         * @param options ros2 node options.
         */
        explicit TrajectoryGenerator(const rclcpp::NodeOptions & options);

    private:

         /** State message callback
         * 
         * Updates private variable containing the AUV's current pose. 
         * 
         * @param msg geometry pose message with position and orientation
         */
        void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

         /** Trajectory Type message callback
         * 
         * Updates private variable containing the current trajectory type. 
         * 
         * @param msg triton_interfaces trajectory type message with type
         */
        void type_callback(const triton_interfaces::msg::TrajectoryType::SharedPtr msg);

         /** Gate pose message callback
         * 
         * Updates private variable containing the gate's current pose (in the base_link frame)
         * 
         * @param msg triton_interfaces object offset message with pose and object type
         */
        void gate_callback(const triton_interfaces::msg::ObjectOffset::SharedPtr msg);

         /** Waypoint message callback
         * 
         * Checks if current waypoint is completed
         * 
         * @param msg triton_interfaces waypoint message with pose and waypoint type
         */
        void waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg);

        // Publish waypoint 
        rclcpp::Publisher<triton_interfaces::msg::Waypoint>::SharedPtr waypoint_publisher_;
        // Current state of AUV
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;
        // Trajectory type
        rclcpp::Subscription<triton_interfaces::msg::TrajectoryType>::SharedPtr type_subscription_;
        // Gate pose
        rclcpp::Subscription<triton_interfaces::msg::ObjectOffset>::SharedPtr gate_subscription_;
        // Current Waypoint
        rclcpp::Subscription<triton_interfaces::msg::Waypoint>::SharedPtr waypoint_subscription_;

        uint8_t type_;
        geometry_msgs::msg::Pose current_pose_;     // AUV current pose
        geometry_msgs::msg::Pose destination_pose_;     
        std::vector<triton_interfaces::msg::Waypoint> waypoints_; // Destination waypoints in trajectory 
        bool destination_achieved_;                    

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::TrajectoryGenerator)

#endif  //TRITON_CONTROL__TRAJECTORY_GENERATOR
