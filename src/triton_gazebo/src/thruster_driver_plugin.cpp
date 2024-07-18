#include "triton_gazebo/thruster_driver_plugin.hpp"

namespace triton_gazebo
{

    ThrusterDriver::ThrusterDriver() : node{rclcpp::Node::make_shared("thruster_driver")} {}


    ThrusterDriver::~ThrusterDriver() {}


    void ThrusterDriver::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        if (_sdf->HasElement("thruster_count"))
        {
            this->thruster_count = _sdf->Get<unsigned int>("thruster_count");
        }
        else
        {
            gzerr << "thruster_count value not specified, exiting.\n";
            exit(1);
        }

        sdf::ElementPtr ros_namespace = _sdf->GetElement("ros");
        this->GetRosNamespace(ros_namespace);

        this->thrust_values = std::vector<double>(this->thruster_count, 0);
        this->force_cmd = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                            this->topic_name, 
                            10, 
                            std::bind(&ThrusterDriver::GetForceCmd, this, _1));

        RCLCPP_INFO(node->get_logger(), "Listening on " + this->topic_name + "\n");

        std::string model_name = _model->GetName();

        for (unsigned int i = 1; i <= thruster_count; i++)
        {
            std::string thruster_name = model_name + "::thruster" + std::to_string(i) + "::thruster";
            this->thruster.push_back(_model->GetLink(thruster_name));

            RCLCPP_INFO(this->node->get_logger(), this->thruster[i-1]->GetName());
        }
    
        this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                                    std::bind(&ThrusterDriver::ApplyForce, this));

        /// @todo feels like there should be a way to pass rclcpp::spin directly to thread, this works the way it is though 
        this->spinThread = std::thread(std::bind(&ThrusterDriver::SpinNode, this));
    }


    void ThrusterDriver::GetRosNamespace(sdf::ElementPtr ros_sdf)
    {
        std::string _namespace;
        std::string topic;
    
        if (ros_sdf->HasElement("namespace"))
        {
            _namespace = ros_sdf->Get<std::string>("namespace");
        }
        else
        {
            _namespace = "triton/triton_gazebo";
        }

        if (ros_sdf->HasElement("remapping"))
        {
            topic = ros_sdf->Get<std::string>("remapping");
        }
        else
        {
            topic = "thruster_values";
        }

        this->topic_name = _namespace + "/" + topic;
    }


    void ThrusterDriver::GetForceCmd(const std_msgs::msg::Float64MultiArray::SharedPtr joint_cmd)
    {
        if (joint_cmd->data.size() != this->thruster_count)
        {
            RCLCPP_WARN(node->get_logger(), "message size does not match thruster count, ignoring command.\n");
            return;
        }

        for (unsigned int i = 0; i < this->thruster_count; i++)
        {
            this->thrust_values[i] = joint_cmd->data[i];
        }
    }


    void ThrusterDriver::ApplyForce()
    {
        for (unsigned int i = 0; i < thruster_count; i++)
        {
            this->thruster[i]->AddLinkForce(ignition::math::Vector3d(0, 0, this->thrust_values[i]));
        }
    }


    void ThrusterDriver::SpinNode()
    {
        rclcpp::spin(node);
    }

}
