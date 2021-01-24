#ifndef _THRUSTER_DRIVER_PLUGIN_HH_
#define _THRUSTER_DRIVER_PLUGIN_HH_

#include <vector> 

// Gazebo libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// ROS2 libraries
#include "rclcpp/rclcpp.hpp"

// Msgs
#include "std_msgs/msg/float64_multi_array.hpp"

namespace triton_gazebo
{

    using std::placeholders::_1;

    class ThrusterDriver : public gazebo::ModelPlugin
    {
    private:
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_cmd;
        gazebo::physics::ModelPtr model;

        std::vector<gazebo::physics::LinkPtr> thruster;
        std::thread spinThread;
        std::string topic_name;

        void SpinNode(void);
        void TorqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr) const;
    public: 
        ThrusterDriver(void);
        ~ThrusterDriver(void);
        virtual void Load(gazebo::physics::ModelPtr, sdf::ElementPtr);
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(ThrusterDriver)

}
#endif // _THRUSTER_DRIVER_PLUGIN_HH_