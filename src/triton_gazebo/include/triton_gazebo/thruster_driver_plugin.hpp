#ifndef _THRUSTER_DRIVER_PLUGIN_HH_
#define _THRUSTER_DRIVER_PLUGIN_HH_

#include <vector> 
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace triton_gazebo
{
    using std::placeholders::_1;

    class ThrusterDriver : public gazebo::ModelPlugin
    {
    private:
        int array_size;
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_cmd;
        std::vector<gazebo::physics::LinkPtr> thruster;
        std::vector<double> thrust_values;
        gazebo::event::ConnectionPtr updateConnection_;

        std::thread spinThread;
        std::string topic_name;

        void SpinNode(void);
        void GetForceCmd(const std_msgs::msg::Float64MultiArray::SharedPtr);
        void ApplyForce(void);
    public: 
        ThrusterDriver(void);
        ~ThrusterDriver(void);
        virtual void Load(gazebo::physics::ModelPtr, sdf::ElementPtr);
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(ThrusterDriver)

}
#endif // _THRUSTER_DRIVER_PLUGIN_HH_