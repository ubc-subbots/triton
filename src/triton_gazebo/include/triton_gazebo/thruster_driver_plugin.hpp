#ifndef TRITON_GAZEBO__THRUSTER_DRIVER_PLUGIN
#define TRITON_GAZEBO__THRUSTER_DRIVER_PLUGIN

#include <vector> 

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace triton_gazebo
{
    /* Function descriptions found in  cpp*/

    /**
     * The GetForceCmd function will be called continuously and recieve a vector from the thrust allocation node,
     * we need to read that value and pass the each force value to the correct thruster
     */


    /** 
     * The ApplyForce function will update in sequence with gazebo's main world update function
     */

    /** 
     * Spin is a bloacking function, to allow the node to run with Gazebo we need to spin on a dedicated thread
     */

    /**
     * The load function for this plugin
     * 
     * Collects all neccessary parameters and initializes the ROS 2 node.
     * 
     * @param _sdf   A pointer to the robot's SDF description
     * @param _model A pointer to the attached mdoel
     * 
     */

    using std::placeholders::_1;

    class ThrusterDriver : public gazebo::ModelPlugin
    {
    private:
        int thruster_count;
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

    GZ_REGISTER_MODEL_PLUGIN(ThrusterDriver)

}
#endif // TRITON_GAZEBO__THRUSTER_DRIVER_PLUGIN