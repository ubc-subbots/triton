#ifndef TRITON_MISSION_PLANNER__MISSION_PLANNER_ROOT
#define TRITON_MISSION_PLANNER__MISSION_PLANNER_ROOT

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include "image_transport/image_transport.hpp"
#include "triton_interfaces/msg/detection_box_array.hpp"

#include "triton_interfaces/msg/trajectory_type.hpp"
#include "triton_interfaces/msg/object_offset.hpp"

#include "triton_mission_planner/gate_tree.hpp" //! probably excessive

namespace triton_mission_planner
{


  class MissionPlanner : public rclcpp::Node
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
      explicit MissionPlanner(const rclcpp::NodeOptions& options);


      /** Registers BehvaiorTree nodes.
       * 
       * Calls functions to register nodes for all BehaviorTree subtrees.
       * 
       * @param factory instance that will generate the tree
       * 
       */
      void registerNodes(BT::BehaviorTreeFactory& factory);


      /** Initialises all ROS 2 publishers.
       * 
       * Creates all of the publishers that will be used by the node.
       * 
       */
      void initPubs();


      /** Initialises all ROS 2 subscribers.
       * 
       * Creates all of the subscribers that will be used by the node.
       * 
       */
      void initSubs();


      /** Publishes on the object recognition topic.
       * 
       * Passes the data given to the object recognition publisher,
       * which publishes it.
       * 
       * @param msg the message to be published.
       * 
       */
      void objectRecognitionPub(const sensor_msgs::msg::Image msg);


      /** Callback function for the object recognition topic.
       * 
       * Updates the corresponding variable with the most recent info
       * published on the topic.
       * 
       * @param msg the message received
       * 
       */
      void objectRecognitionSub(const triton_interfaces::msg::DetectionBoxArray::SharedPtr msg);


      triton_interfaces::msg::DetectionBoxArray::SharedPtr getObjectRecognitionSubInfo();


      void trajectoryGenerationTypePub(triton_interfaces::msg::TrajectoryType msg);

      
      void trajectoryGenerationGatePub(const triton_interfaces::msg::ObjectOffset msg);


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


      //* object recognition
      // pub camera
      image_transport::Publisher obj_rec_pub_;

      // sub: bounding box
      rclcpp::Subscription<triton_interfaces::msg::DetectionBoxArray>::SharedPtr obj_rec_sub_;

      triton_interfaces::msg::DetectionBoxArray::SharedPtr obj_rec_info_;

      //* trajectory generation
      // pub: type
      rclcpp::Publisher<triton_interfaces::msg::TrajectoryType>::SharedPtr traj_gen_type_pub_;

      // pub: gate
      rclcpp::Publisher<triton_interfaces::msg::ObjectOffset>::SharedPtr traj_gen_gate_pub_;
  };


} // namespace triton_mission_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_mission_planner::MissionPlanner)

#endif // TRITON_MISSION_PLANNER__MISSION_PLANNER_ROOT