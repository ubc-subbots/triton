#include "triton_mission_planner/torpedo_tree.hpp"
using std::placeholders::_1;

#include "triton_mission_planner/mission_planner_root.hpp"

namespace triton_mission_planner
{


  TargetIsVisible::TargetIsVisible(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList TargetIsVisible::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus TargetIsVisible::tick()
  {
    //TODO: launch object recognition
    //! only once
    //TODO: test for gate -> update isVisible

    // object type: box.cls -> tensor([16.])
    // coordinates: box.xyxy -> tensor([[1.0000,2.0000,3.0000,4.0000]])
    // probability: box.conf -> tensor([0.9528])

    bool isVisible = false;
    // const int32_t GATE_ID = 0;

    // triton_interfaces::msg::DetectionBoxArray detBoxArr = mp_.getObjectRecognitionSubInfo();

    // for (int i = 0; i < sizeof(detBoxArr.boxes) / sizeof(int); i ++)
    // {
    //   isVisible = detBoxArr.boxes[i].class_id == GATE_ID ? true : false; //! needs to differentiate between objects
    // }

    return isVisible ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }


  TargetFind::TargetFind(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList TargetFind::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus TargetFind::onStart()
  {
    //TODO: launch trajectory generation
    //TODO: launch thrust allocation
    //TODO: launch pid controller
    
    return TargetFind::onRunning(); // calls onRunning to do common actions
  }


  BT::NodeStatus TargetFind::onRunning()
  {
    triton_interfaces::msg::TrajectoryType msg;
    msg.set__type(triton_interfaces::msg::TrajectoryType::START);

    mp_->trajectoryGenerationTypePub(msg);

    return BT::NodeStatus::RUNNING; // will be halted as soon as TargetIsVisible returns SUCCESS
  }


  void TargetFind::onHalted()
  {
    //TODO: halt trajectory generation
    //TODO: halt thrust allocation
    //TODO: halt pid controller

    //? need to create helper functions to do this
  }


  // TargetIsAligned::TargetIsAligned(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  // {}


  // BT::PortsList TargetIsAligned::providedPorts() // needed for compilation
  // {
  //   return {};
  // }


  // BT::NodeStatus TargetIsAligned::tick()
  // {
  //   //TODO:

  //   return BT::NodeStatus::FAILURE;
  // }


  // TargetAlign::TargetAlign(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  // {}


  // BT::PortsList TargetAlign::providedPorts() // needed for compilation
  // {
  //   return {};
  // }


  // BT::NodeStatus TargetAlign::onStart()
  // {
  //   //TODO: launch trajectory generation
  //   //TODO: launch thrust allocation
  //   //TODO: launch pid controller
    
  //   return TargetAlign::onRunning(); // calls onRunning to do common actions
  // }


  // BT::NodeStatus TargetAlign::onRunning()
  // {
  //   triton_interfaces::msg::TrajectoryType msg; //! might need to move to onStart
  //   msg.set__type(triton_interfaces::msg::TrajectoryType::GATE); //? correct message type?

  //   mp_->trajectoryGenerationTypePub(msg);

  //   return BT::NodeStatus::RUNNING; // will be halted as soon as TargetIsAligned returns SUCCESS
  // }


  // void TargetAlign::onHalted()
  // {
  //   //TODO: halt trajectory generation
  //   //TODO: halt thrust allocation
  //   //TODO: halt pid controller
  // }


  TargetShoot::TargetShoot(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList TargetShoot::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus TargetShoot::onStart()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  BT::NodeStatus TargetShoot::onRunning()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  void TargetShoot::onHalted()
  {
    //TODO:
  }


} // namespace triton_mission_planner