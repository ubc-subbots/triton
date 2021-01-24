#include "triton_gazebo/thruster_driver_plugin.hpp"

namespace triton_gazebo
{

    // Constructor
    ThrusterDriver::ThrusterDriver()
    {
        if (!rclcpp::ok())
        {
            int argc = 0; 
            char **argv = NULL;
            rclcpp::init(argc, argv);
        }

        node = rclcpp::Node::make_shared("thruster_driver");
    }

    // Destructor 
    ThrusterDriver::~ThrusterDriver() {}

    /**
     * This function will be called continuously and recieve a vector from the thrust allocation node,
     * we need to read that value and pass the each force value to the correct thruster
     */
    void ThrusterDriver::TorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr joint_cmd) const
    {

    }

    /** 
     * Spin is a bloacking function, to allow the node to run with Gazebo we need to spin on a dedicated thread
     */
    void ThrusterDriver::SpinNode()
    {
        rclcpp::spin(node);
    }

    /**
     * The load function for this plugin
     * 
     * Collects all neccessary parameters and initializes the ROS 2 node.
     * 
     * @param _sdf   A pointer to the robot's SDF description
     * @param _model A pointer to the attached mdoel
     * 
     */
    void ThrusterDriver::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        if (_sdf->HasElement("topic_name"))
        {
            topic_name = _sdf->Get<std::string>("topic_name");
        }
        else
        {
            topic_name = "triton_gazebo/thruster_vector";
        }
        
        force_cmd = node->create_subscription<std_msgs::msg::Flsoat64MultiArray>(
                        topic_name, 
                        10, 
                        std::bind(&ThrusterDriver::TorqueCallback, this, _1));

        RCLCPP_INFO(node->get_logger(), "Subscriber node created.\n");

        /// @todo feels like there should be a way to pass rclcpp::spin directly to thread, this works the way it is though 
        spinThread = std::thread(std::bind(&ThrusterDriver::SpinNode, this));
    }

}