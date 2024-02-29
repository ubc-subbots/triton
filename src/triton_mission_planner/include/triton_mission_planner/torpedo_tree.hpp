#ifndef TRITON_MISSION_PLANNER__TORPEDO_TREE
#define TRITON_MISSION_PLANNER__TORPEDO_TREE

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include "image_transport/image_transport.hpp"
#include "triton_interfaces/msg/detection_box_array.hpp"

#include "triton_interfaces/msg/trajectory_type.hpp"
#include "triton_interfaces/msg/object_offset.hpp"

namespace triton_mission_planner
{


  class MissionPlanner;


  class TargetIsVisible : public BT::ConditionNode
  {
    public:


      /** Constructor for the target visibility check condition.
       * 
       * @param name the name of the instance.
       * @param config behavior tree options (unused because no blackboard ports),
       * but needed in order to add an additional argument.
       * @param rosnode the ros 2 node for it to reference to run ros methods.
       * 
       * @note can't use a simpleConditionNode because we need to pass
       * the mission planner node.
       * 
       */
      TargetIsVisible(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);


      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus tick() override;


    private:

      MissionPlanner* mp_;

  };


  class TargetFind : public BT::StatefulActionNode
  {
    public:


      TargetFind(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);

      
      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;

    
    private:

      MissionPlanner* mp_;

  };


  class TargetIsAligned : public BT::ConditionNode
  {
    public:


      TargetIsAligned(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);


      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus tick() override;


    private:

      MissionPlanner* mp_;

  };


  class TargetAlign : public BT::StatefulActionNode
  {
    public:


      TargetAlign(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);


      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;


    private:

      MissionPlanner* mp_;
      
  };


  class TargetShoot : public BT::StatefulActionNode
  {
    public:


      TargetShoot(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);


      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;


    private:

      MissionPlanner* mp_;

  };


} // namespace triton_mission_planner

#endif // TRITON_MISSION_PLANNER__TORPEDO_TREE