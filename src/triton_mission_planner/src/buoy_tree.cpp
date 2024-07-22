#include "triton_mission_planner/buoy_tree.hpp"
using std::placeholders::_1;

#include "triton_mission_planner/mission_planner_root.hpp"

namespace triton_mission_planner
{


  BuoyIsVisible::BuoyIsVisible(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList BuoyIsVisible::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus BuoyIsVisible::tick()
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


  BuoyFind::BuoyFind(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList BuoyFind::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus BuoyFind::onStart()
  {
    //TODO: launch trajectory generation
    //TODO: launch thrust allocation
    //TODO: launch pid controller
    
    return BuoyFind::onRunning(); // calls onRunning to do common actions
  }


  BT::NodeStatus BuoyFind::onRunning()
  {
    triton_interfaces::msg::TrajectoryType msg;
    msg.set__type(triton_interfaces::msg::TrajectoryType::START);

    mp_->trajectoryGenerationTypePub(msg);

    return BT::NodeStatus::RUNNING; // will be halted as soon as BuoyIsVisible returns SUCCESS
  }


  void BuoyFind::onHalted()
  {
    //TODO: halt trajectory generation
    //TODO: halt thrust allocation
    //TODO: halt pid controller

    //? need to create helper functions to do this
  }


  // BuoyIsAligned::BuoyIsAligned(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  // {}


  // BT::PortsList BuoyIsAligned::providedPorts() // needed for compilation
  // {
  //   return {};
  // }


  // BT::NodeStatus BuoyIsAligned::tick()
  // {
  //   //TODO:

  //   return BT::NodeStatus::FAILURE;
  // }


  // BuoyAlign::BuoyAlign(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  // {}


  // BT::PortsList BuoyAlign::providedPorts() // needed for compilation
  // {
  //   return {};
  // }


  // BT::NodeStatus BuoyAlign::onStart()
  // {
  //   //TODO: launch trajectory generation
  //   //TODO: launch thrust allocation
  //   //TODO: launch pid controller
    
  //   return BuoyAlign::onRunning(); // calls onRunning to do common actions
  // }


  // BT::NodeStatus BuoyAlign::onRunning()
  // {
  //   triton_interfaces::msg::TrajectoryType msg; //! might need to move to onStart
  //   msg.set__type(triton_interfaces::msg::TrajectoryType::GATE); //? correct message type?

  //   mp_->trajectoryGenerationTypePub(msg);

  //   return BT::NodeStatus::RUNNING; // will be halted as soon as BuoyIsAligned returns SUCCESS
  // }


  // void BuoyAlign::onHalted()
  // {
  //   //TODO: halt trajectory generation
  //   //TODO: halt thrust allocation
  //   //TODO: halt pid controller
  // }


  BuoyHit::BuoyHit(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList BuoyHit::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus BuoyHit::onStart()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  BT::NodeStatus BuoyHit::onRunning()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  void BuoyHit::onHalted()
  {
    //TODO:
  }


} // namespace triton_mission_planner