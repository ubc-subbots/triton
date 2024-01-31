#include "triton_mission_planner/mission_planner_root.hpp"
using std::placeholders::_1;

namespace triton_mission_planner
{


  MissionPlanner::MissionPlanner(const rclcpp::NodeOptions& options) : Node("mission_planner_root", options)
  {
    initPubs();
    initSubs();

    BT::BehaviorTreeFactory factory;
    registerNodes(factory);

    factory.registerBehaviorTreeFromText("../config/tree.xml");
    auto tree = factory.createTree("MainTree");

    tree.tickWhileRunning();
  }


  void MissionPlanner::initPubs()
  {
    // object recognition
    obj_rec_pub_ = image_transport::create_publisher(this, "object_recognizer/in");

    // trajectory generation
    traj_gen_type_pub_ = this->create_publisher<triton_interfaces::msg::TrajectoryType>("/triton/controls/trajectory_generator/set_type", 10);

    traj_gen_gate_pub_ = this->create_publisher<triton_interfaces::msg::ObjectOffset>("/triton/gate/detector/gate_pose", 10);
  }


  void MissionPlanner::initSubs()
  {
    // object recognition
    obj_rec_sub_ = this->create_subscription<triton_interfaces::msg::DetectionBoxArray>("object_recognizer/out", 10, std::bind(&MissionPlanner::objectRecognitionSub, this, _1));
  }


  void MissionPlanner::objectRecognitionPub(const sensor_msgs::msg::Image msg)
  {
    obj_rec_pub_.publish(msg); //! check if dot operator correct
  }


  void MissionPlanner::objectRecognitionSub(const triton_interfaces::msg::DetectionBoxArray::SharedPtr msg)
  {
    obj_rec_info_ = msg;
  }


  triton_interfaces::msg::DetectionBoxArray::SharedPtr MissionPlanner::getObjectRecognitionSubInfo()
  {
    return obj_rec_info_;
  }


  void MissionPlanner::trajectoryGenerationTypePub(triton_interfaces::msg::TrajectoryType msg)
  {
    traj_gen_type_pub_->publish(msg);
  }

      
  void MissionPlanner::trajectoryGenerationGatePub(const triton_interfaces::msg::ObjectOffset msg)
  {
    traj_gen_gate_pub_->publish(msg);
  }
  

  void MissionPlanner::registerNodes(BT::BehaviorTreeFactory& factory)
  {
    //* gate nodes
    // custom action nodes
    factory.registerNodeType<GateFind>("GateFind", this);
    factory.registerNodeType<GateAlign>("GateAlign", this);
    factory.registerNodeType<GateGoThrough>("GateGoThrough", this);

    //custom condition nodes
    factory.registerNodeType<GateIsVisible>("GateIsVisble", this);
    factory.registerNodeType<GateIsAligned>("GateIsAligned", this);

    //TODO: add other registers
  }


} // namespace triton_mission_planner

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<triton_mission_planner::MissionPlanner>(options));
  rclcpp::shutdown();
  return 0;
}