#include "triton_mission_planner/mission_planner_root.hpp"
using std::placeholders::_1;

#include "triton_mission_planner/gate_tree.hpp"
#include "triton_mission_planner/path_tree.hpp"
#include "triton_mission_planner/buoy_tree.hpp"
#include "triton_mission_planner/bin_tree.hpp"
#include "triton_mission_planner/torpedo_tree.hpp"

namespace triton_mission_planner
{


  MissionPlanner::MissionPlanner(const rclcpp::NodeOptions& options) : Node("mission_planner_root", options)
  {
    initPubs();
    initSubs();

    BT::BehaviorTreeFactory factory;

    const int PRE_COMP = 0;

    if (PRE_COMP == 1)
    {
      // registerPreCompNodes(factory);
    }
    else
    {
      registerNodes(factory);
    }

    // Retrieve the parameter for the file path
    std::string tree_file_path;
    this->declare_parameter<std::string>("tree_file_path", "");
    this->get_parameter("tree_file_path", tree_file_path);

    if (tree_file_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "tree_file_path parameter is empty. Cannot load behavior tree.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading behavior tree from: %s", tree_file_path.c_str());


     factory.registerBehaviorTreeFromFile(tree_file_path);
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
    // factory.registerNodeType<GateAlign>("GateAlign", this);
    factory.registerNodeType<GateGoThrough>("GateGoThrough", this);

    //custom condition nodes
    factory.registerNodeType<GateIsVisible>("GateIsVisble", this);
    // factory.registerNodeType<GateIsAligned>("GateIsAligned", this);

    //* path nodes
    //custom action nodes
    factory.registerNodeType<PathFind>("PathFind", this);
    // factory.registerNodeType<PathAlign>("PathAlign", this);
    factory.registerNodeType<PathFollow>("PathFollow", this);

    //custom condition nodes
    factory.registerNodeType<PathIsVisible>("PathIsVisble", this);
    // factory.registerNodeType<PathIsAligned>("PathIsAligned", this);

    //* buoy nodes
    //custom action nodes
    factory.registerNodeType<BuoyFind>("BuoyFind", this);
    // factory.registerNodeType<BuoyAlign>("BuoyAlign", this);
    factory.registerNodeType<BuoyHit>("BuoyHit", this);

    //custom condition nodes
    factory.registerNodeType<BuoyIsVisible>("BuoyIsVisble", this);
    // factory.registerNodeType<BuoyIsAligned>("BuoyIsAligned", this);

    //* bin nodes
    //custom action nodes
    factory.registerNodeType<BinFind>("BinFind", this);
    factory.registerNodeType<BinAlign>("BinAlign", this);
    factory.registerNodeType<BinAlignAbove>("BinAlignAbove", this);
    factory.registerNodeType<BinDrop>("BinDrop", this);

    //custom condition nodes
    factory.registerNodeType<BinIsVisible>("BinIsVisble", this);
    factory.registerNodeType<BinIsAligned>("BinIsAligned", this);

    //* torpedo nodes
    //custom action nodes
    factory.registerNodeType<TargetFind>("TargetFind", this);
    // factory.registerNodeType<TargetAlign>("TargetAlign", this);
    factory.registerNodeType<TargetShoot>("TargetShoot", this);

    //custom condition nodes
    factory.registerNodeType<TargetIsVisible>("TargetIsVisble", this);
    // factory.registerNodeType<TargetIsAligned>("TargetIsAligned", this);
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
