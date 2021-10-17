#ifndef TRITON_CONTROL__WAYPOINT_MARKER
#define TRITON_CONTROL__WAYPOINT_MARKER

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "triton_interfaces/msg/pipeline_feedback.hpp"
//#include "triton_control/waypoint_sequence_manager.hpp" // Where enum is currently stored

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

         /** Pose message callback
         * 
         * Publishes the waypoint position calculated by taking the waypoint
         * and adding the initial position given in the first message recieved.
         * 
         * @param msg geometry pose message witth position and orientation
         */
        void subscriberCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void serviceCallback(const triton_interfaces::srv::MarkWaypoint::Request::SharedPtr request,
                             const triton_interfaces::srv::MarkWaypoint::Response::SharedPtr response) const;

        rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> & parameters);

//        rclcpp::Publisher<triton_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
        rclcpp::Publisher<triton_interfaces::msg::Success>::SharedPtr success_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
        rclcpp::Service<triton_interfaces::srv::MarkWaypoint>::SharedPtr service_;

        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        bool has_pose_;
        geometry_msgs::msg::Pose pose_;

        bool has_waypoint_;

        /* Order of values:
        * Point, then quaternion
        * x, y, z,
        * x, y, z, w
        */
        std::vector<double> waypoint_pose_values_;
        std::string threshold_type_;
        /* Order of values:
        * Distance, then angle difference
        */
        std::vector<double> waypoint_thresh_values_;
//        double threshold_dist_;
//        double threshold_ang_diff_;
        const int threshold_consect_ = 10;
        const int wait_time_ = 500 000 000; // 500 million or 0.5 seconds

        int threshold_counter_;

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::WaypointMarker)

#endif  //TRITON_CONTROL__WAYPOINT_MARKER
