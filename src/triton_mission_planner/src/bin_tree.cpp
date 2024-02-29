#include "triton_mission_planner/bin_tree.hpp"
using std::placeholders::_1;

#include "triton_mission_planner/mission_planner_root.hpp"

namespace triton_mission_planner
{


  BinIsVisible::BinIsVisible(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList BinIsVisible::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus BinIsVisible::tick()
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


  BinFind::BinFind(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList BinFind::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus BinFind::onStart()
  {
    //TODO: launch trajectory generation
    //TODO: launch thrust allocation
    //TODO: launch pid controller
    
    return BinFind::onRunning(); // calls onRunning to do common actions
  }


  BT::NodeStatus BinFind::onRunning()
  {
    triton_interfaces::msg::TrajectoryType msg;
    msg.set__type(triton_interfaces::msg::TrajectoryType::START);

    mp_->trajectoryGenerationTypePub(msg);

    return BT::NodeStatus::RUNNING; // will be halted as soon as BinIsVisible returns SUCCESS
  }


  void BinFind::onHalted()
  {
    //TODO: halt trajectory generation
    //TODO: halt thrust allocation
    //TODO: halt pid controller

    //? need to create helper functions to do this
  }


  BinIsAligned::BinIsAligned(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : ConditionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList BinIsAligned::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus BinIsAligned::tick()
  {
    //TODO:

    return BT::NodeStatus::FAILURE;
  }


  BinAlign::BinAlign(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList BinAlign::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus BinAlign::onStart()
  {
    //TODO: launch trajectory generation
    //TODO: launch thrust allocation
    //TODO: launch pid controller
    
    return BinAlign::onRunning(); // calls onRunning to do common actions
  }


  BT::NodeStatus BinAlign::onRunning()
  {
    triton_interfaces::msg::TrajectoryType msg; //! might need to move to onStart
    msg.set__type(triton_interfaces::msg::TrajectoryType::GATE); //? correct message type?

    mp_->trajectoryGenerationTypePub(msg);

    return BT::NodeStatus::RUNNING; // will be halted as soon as BinIsAligned returns SUCCESS
  }


  void BinAlign::onHalted()
  {
    //TODO: halt trajectory generation
    //TODO: halt thrust allocation
    //TODO: halt pid controller
  }


  BinAlignAbove::BinAlignAbove(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList BinAlignAbove::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus BinAlignAbove::onStart()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  BT::NodeStatus BinAlignAbove::onRunning()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  void BinAlignAbove::onHalted()
  {
    //TODO:
  }


  BinDrop::BinDrop(const std::string& name, const BT::NodeConfig& config, MissionPlanner* rosnode) : StatefulActionNode(name, config), mp_(rosnode)
  {}


  BT::PortsList BinDrop::providedPorts() // needed for compilation
  {
    return {};
  }


  BT::NodeStatus BinDrop::onStart()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  BT::NodeStatus BinDrop::onRunning()
  {
    //TODO:

    return BT::NodeStatus::RUNNING;
  }


  void BinDrop::onHalted()
  {
    //TODO:
  }


} // namespace triton_mission_planner