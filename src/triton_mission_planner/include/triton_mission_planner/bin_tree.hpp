#ifndef TRITON_MISSION_PLANNER__BIN_TREE
#define TRITON_MISSION_PLANNER__BIN_TREE

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


  class BinIsVisible : public BT::ConditionNode
  {
    public:


      /** Constructor for the bin visibility check condition.
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
      BinIsVisible(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);


      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus tick() override;


    private:

      MissionPlanner* mp_;

  };


  class BinFind : public BT::StatefulActionNode
  {
    public:


      BinFind(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);

      
      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;

    
    private:

      MissionPlanner* mp_;

  };


  class BinIsAligned : public BT::ConditionNode
  {
    public:


      BinIsAligned(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);


      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus tick() override;


    private:

      MissionPlanner* mp_;

  };


  class BinAlign : public BT::StatefulActionNode
  {
    public:


      BinAlign(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);


      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;


    private:

      MissionPlanner* mp_;
      
  };


  class BinAlignAbove : public BT::StatefulActionNode
  {
    public:


      BinAlignAbove(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);


      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;


    private:

      MissionPlanner* mp_;

  };


  class BinDrop : public BT::StatefulActionNode
  {
    public:


      BinDrop(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode);


      static BT::PortsList providedPorts(); // needed for compilation


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;


    private:

      MissionPlanner* mp_;

  };


} // namespace triton_mission_planner

#endif // TRITON_MISSION_PLANNER__BIN_TREE