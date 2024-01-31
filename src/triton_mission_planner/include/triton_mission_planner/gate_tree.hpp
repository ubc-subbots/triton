#ifndef TRITON_MISSION_PLANNER__GATE_TREE
#define TRITON_MISSION_PLANNER__GATE_TREE

#include "triton_mission_planner/mission_planner_root.hpp"

namespace triton_mission_planner
{


  class GateIsVisible : public BT::SyncActionNode
  {
    public:


      /** Constructor for the gate visibility check condition.
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
      GateIsVisible(const std::string& name, const BT::NodeConfig& config, MissionPlanner& rosnode);


      BT::NodeStatus tick() override;


    private:

      MissionPlanner& mp_;

  };


  class GateFind : public BT::StatefulActionNode
  {
    public:


      GateFind(const std::string& name, const BT::NodeConfig& config, MissionPlanner& rosnode);

      
      // static BT::PortsList providedPorts(); //? may be needed?


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;

    
    private:

      MissionPlanner& mp_;

  };


  class GateIsAligned : public BT::SyncActionNode
  {
    public:


      GateIsAligned(const std::string& name, const BT::NodeConfig& config, MissionPlanner& rosnode);


      BT::NodeStatus tick() override;


    private:

      MissionPlanner& mp_;

  };


  class GateAlign : public BT::StatefulActionNode
  {
    public:


      GateAlign(const std::string& name, const BT::NodeConfig& config, MissionPlanner& rosnode);


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;


    private:

      MissionPlanner& mp_;
      
  };


  class GateGoThrough : public BT::StatefulActionNode
  {
    public:


      GateGoThrough(const std::string& name, const BT::NodeConfig& config, MissionPlanner& rosnode);


      BT::NodeStatus onStart() override;


      BT::NodeStatus onRunning() override;


      void onHalted() override;


    private:

      MissionPlanner& mp_;

  };


} // namespace triton_mission_planner

#endif // TRITON_MISSION_PLANNER__GATE_TREE