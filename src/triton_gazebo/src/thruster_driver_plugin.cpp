#include "triton_gazebo/thruster_driver_plugin.hpp"

namespace triton_gazebo
{

    // Constructor
    ThrusterDriver::ThrusterDriver() : node{rclcpp::Node::make_shared("thruster_driver")} {}

    // Destructor 
    ThrusterDriver::~ThrusterDriver() {}

    /**
     * This function will be called continuously and recieve a vector from the thrust allocation node,
     * we need to read that value and pass the each force value to the correct thruster
     */
    void ThrusterDriver::GetForceCmd(const std_msgs::msg::Float64MultiArray::SharedPtr joint_cmd)
    {
        int array_size = joint_cmd->data.size();
        RCLCPP_INFO(node->get_logger(), "recieved vector of size: %f\n", array_size);

        for (int i = 0; i < array_size; i++)
        {
            RCLCPP_INFO(node->get_logger(), "%f ", joint_cmd->data[i]);
            thrust_values[i] = joint_cmd->data[i];
        }
    }

    /** 
     * This function will update in sequence with gazebo's main world update function
     */
    void ThrusterDriver::ApplyForce()
    {
        for (int i = 0; i < 2; i++)
        {
            thruster[i]->SetForce(ignition::math::Vector3d(0, 0, thrust_values[i]));
        }
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
        thrust_values = std::vector<double>(6, 0);
        force_cmd = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                        topic_name, 
                        10, 
                        std::bind(&ThrusterDriver::GetForceCmd, this, _1));

        RCLCPP_INFO(node->get_logger(), "Subscriber node created.\n");

        thruster.push_back(_model->GetLink("triton_auv::thruster5::thruster"));
        thruster.push_back(_model->GetLink("triton_auv::thruster6::thruster"));

        RCLCPP_INFO(node->get_logger(), thruster[0]->GetName());

        updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                                std::bind(&ThrusterDriver::ApplyForce, this));

        /// @todo feels like there should be a way to pass rclcpp::spin directly to thread, this works the way it is though 
        spinThread = std::thread(std::bind(&ThrusterDriver::SpinNode, this));
    }

}