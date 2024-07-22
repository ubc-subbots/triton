#include "triton_mission_planner/path_tree.hpp"
using std::placeholders::_1;

#include "triton_mission_planner/mission_planner_root.hpp"

namespace triton_mission_planner
{


  PathIsVisible::PathIsVisible(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList PathIsVisible::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus PathIsVisible::tick()
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


  PathFind::PathFind(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList PathFind::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus PathFind::onStart()
  {
    //TODO: launch trajectory generation
    //TODO: launch thrust allocation
    //TODO: launch pid controller
    
    return PathFind::onRunning(); // calls onRunning to do common actions
  }


  BT::NodeStatus PathFind::onRunning()
  {
    triton_interfaces::msg::TrajectoryType msg;
    msg.set__type(triton_interfaces::msg::TrajectoryType::START);

    mp_->trajectoryGenerationTypePub(msg);

    return BT::NodeStatus::RUNNING; // will be halted as soon as PathIsVisible returns SUCCESS
  }


  void PathFind::onHalted()
  {
    //TODO: halt trajectory generation
    //TODO: halt thrust allocation
    //TODO: halt pid controller

    //? need to create helper functions to do this
  }


  // PathIsAligned::PathIsAligned(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  // {}


  // BT::PortsList PathIsAligned::providedPorts() // needed for compilation
  // {
  //   return {};
  // }


  // BT::NodeStatus PathIsAligned::tick()
  // {
  //   //TODO:

  //   return BT::NodeStatus::FAILURE;
  // }


  // PathAlign::PathAlign(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  // {}


  // BT::PortsList PathAlign::providedPorts() // needed for compilation
  // {
  //   return {};
  // }


  // BT::NodeStatus PathAlign::onStart()
  // {
  //   //TODO: launch trajectory generation
  //   //TODO: launch thrust allocation
  //   //TODO: launch pid controller
    
  //   return PathAlign::onRunning(); // calls onRunning to do common actions
  // }


  // BT::NodeStatus PathAlign::onRunning()
  // {
  //   triton_interfaces::msg::TrajectoryType msg; //! might need to move to onStart
  //   msg.set__type(triton_interfaces::msg::TrajectoryType::GATE); //? correct message type?

  //   mp_->trajectoryGenerationTypePub(msg);

  //   return BT::NodeStatus::RUNNING; // will be halted as soon as PathIsAligned returns SUCCESS
  // }


  // void PathAlign::onHalted()
  // {
  //   //TODO: halt trajectory generation
  //   //TODO: halt thrust allocation
  //   //TODO: halt pid controller
  // }


  PathFollow::PathFollow(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList PathFollow::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus PathFollow::onStart()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  BT::NodeStatus PathFollow::onRunning()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  void PathFollow::onHalted()
  {
    //TODO:
  }


} // namespace triton_mission_planner