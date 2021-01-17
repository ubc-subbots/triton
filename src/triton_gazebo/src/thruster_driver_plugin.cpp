#include "triton_gazebo/thruster_driver_plugin.hpp"

namespace triton_gazebo
{

// Constructor
ThrusterDriver::ThrusterDriver() {}

ThrusterDriver::~ThrusterDriver() {}

void ThrusterDriver::TorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr joint_cmd) const
{

}

// Spin is a bloacking function, to allow the node to run with Gazebo we need to spin on an independant thread
void ThrusterDriver::CallbackThread()
{
    rclcpp::spin(node);
}

/**
 * The load function for this plugin
 * 
 * Collects all neccessary parameters and initializes the ROS2 node.
 * 
 * @param _sdf   A pointer to the robot's SDF description
 * @param _model A pointer to the attached mdoel
 * 
 */
void ThrusterDriver::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{


    torque_command = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                        topic_name, 
                        10, 
                        std::bind(&ThrusterDriver::TorqueCallback, this, _1));

    spinThread = std::thread(std::bind(&ThrusterDriver::CallbackThread, this));
}

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(ThrusterDriver)

}