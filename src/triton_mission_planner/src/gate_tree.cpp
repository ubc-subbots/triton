#include "triton_mission_planner/gate_tree.hpp"
using std::placeholders::_1;

#include "triton_mission_planner/mission_planner_root.hpp"

namespace triton_mission_planner
{


  GateIsVisible::GateIsVisible(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList GateIsVisible::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus GateIsVisible::tick()
  {
    //TODO: launch object recognition
    //! only once
    //TODO: test for gate -> update isVisible

    // object type: box.cls -> tensor([16.])
    // coordinates: box.xyxy -> tensor([[1.0000,2.0000,3.0000,4.0000]])
    // probability: box.conf -> tensor([0.9528])

    const int32_t GATE_ID = 0;

    triton_interfaces::msg::DetectionBoxArray::SharedPtr detBoxArr = mp_->getObjectRecognitionSubInfo();

    for (int i = 0; i < detBoxArr->boxes.size(); i ++)
    {
      if (detBoxArr->boxes[i].class_id == GATE_ID)
      {
        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::FAILURE;
  }


  GateFind::GateFind(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList GateFind::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus GateFind::onStart()
  {
    //TODO: launch trajectory generation
    //TODO: launch thrust allocation
    //TODO: launch pid controller
    
    return GateFind::onRunning(); // calls onRunning to do common actions
  }


  BT::NodeStatus GateFind::onRunning()
  {
    triton_interfaces::msg::TrajectoryType msg;
    msg.set__type(triton_interfaces::msg::TrajectoryType::START);

    mp_->trajectoryGenerationTypePub(msg);

    return BT::NodeStatus::RUNNING; // will be halted as soon as GateIsVisible returns SUCCESS
  }


  void GateFind::onHalted()
  {
    //TODO: halt trajectory generation
    //TODO: halt thrust allocation
    //TODO: halt pid controller

    //? need to create helper functions to do this
  }


  // GateIsAligned::GateIsAligned(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  // {}


  // BT::PortsList GateIsAligned::providedPorts() // needed for compilation
  // {
  //   return {};
  // }


  // BT::NodeStatus GateIsAligned::tick()
  // {
  //   //TODO:
  //   return BT::NodeStatus::FAILURE;
  // }


  // GateAlign::GateAlign(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  // {}


  // BT::PortsList GateAlign::providedPorts() // needed for compilation
  // {
  //   return {};
  // }


  // BT::NodeStatus GateAlign::onStart()
  // {
  //   //TODO: launch trajectory generation
  //   //TODO: launch thrust allocation
  //   //TODO: launch pid controller
    
  //   return GateAlign::onRunning(); // calls onRunning to do common actions
  // }


  // BT::NodeStatus GateAlign::onRunning()
  // {
  //   triton_interfaces::msg::TrajectoryType msg; //! might need to move to onStart
  //   msg.set__type(triton_interfaces::msg::TrajectoryType::GATE); //? correct message type?
  //   mp_->trajectoryGenerationTypePub(msg);

  //   return BT::NodeStatus::RUNNING; // will be halted as soon as GateIsAligned returns SUCCESS
  // }


  // void GateAlign::onHalted()
  // {
  //   //TODO: halt trajectory generation
  //   //TODO: halt thrust allocation
  //   //TODO: halt pid controller
  // }


  GateGoThrough::GateGoThrough(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList GateGoThrough::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus GateGoThrough::onStart()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  BT::NodeStatus GateGoThrough::onRunning()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  void GateGoThrough::onHalted()
  {
    //TODO:
  }


} // namespace triton_mission_planner