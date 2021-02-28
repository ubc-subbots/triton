#include "triton_gazebo/thruster_driver_plugin.hpp"
#include <string.h>

namespace triton_gazebo
{

    // Constructor
    ThrusterDriver::ThrusterDriver() : node{rclcpp::Node::make_shared("thruster_driver")} {}

    // Destructor 
    ThrusterDriver::~ThrusterDriver() {}

    void ThrusterDriver::GetForceCmd(const std_msgs::msg::Float64MultiArray::SharedPtr joint_cmd)
    {
        int array_size = joint_cmd->data.size();
        RCLCPP_INFO(node->get_logger(), "received vector of size: %d\n", array_size);   

        for (int i = 0; i < array_size; i++)
        {
            RCLCPP_INFO(node->get_logger(), "%f Newtons", joint_cmd->data[i]);
            thrust_values[i] = joint_cmd->data[i];
        }
    }

    void ThrusterDriver::ApplyForce()
    {
        for (int i = 0; i < 6; i++)
        {
            thruster[i]->AddLinkForce(ignition::math::Vector3d(0, 0, thrust_values[i]), ignition::math::Vector3d(0, 0, 0));
        }
    }

    void ThrusterDriver::SpinNode()
    {
        rclcpp::spin(node);
    }

    void ThrusterDriver::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        if (_sdf->HasElement("thruster_count"))
        {
           thruster_count = _sdf->Get<int>("thruster_count");
        }
        else
        {
            thruster_count = 0;
        }

        if (_sdf->HasElement("topic_name"))
        {
            topic_name = _sdf->Get<std::string>("topic_name");
        }
        else
        {
            topic_name = "triton_gazebo/thruster_vector";
        }

        thrust_values = std::vector<double>(thruster_count, 0);
        force_cmd = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                        topic_name, 
                        10, 
                        std::bind(&ThrusterDriver::GetForceCmd, this, _1));

        RCLCPP_INFO(node->get_logger(), "Subscriber node created.\n");

        thruster.push_back(_model->GetLink("triton_auv::thruster1::thruster"));
        thruster.push_back(_model->GetLink("triton_auv::thruster2::thruster"));
        thruster.push_back(_model->GetLink("triton_auv::thruster3::thruster"));
        thruster.push_back(_model->GetLink("triton_auv::thruster4::thruster"));
        thruster.push_back(_model->GetLink("triton_auv::thruster5::thruster"));
        thruster.push_back(_model->GetLink("triton_auv::thruster6::thruster"));

        // use the name in the sdf

        for(int i=0; i < thruster_count;i++)
            RCLCPP_INFO(node->get_logger(), thruster[i]->GetName());
    
        updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                                std::bind(&ThrusterDriver::ApplyForce, this));

        /// @todo feels like there should be a way to pass rclcpp::spin directly to thread, this works the way it is though 
        spinThread = std::thread(std::bind(&ThrusterDriver::SpinNode, this));
    }

}