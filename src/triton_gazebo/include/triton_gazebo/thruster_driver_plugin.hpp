#ifndef TRITON_GAZEBO__THRUSTER_DRIVER_PLUGIN
#define TRITON_GAZEBO__THRUSTER_DRIVER_PLUGIN

#include <vector> 

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace triton_gazebo
{
    using std::placeholders::_1;

    class ThrusterDriver : public gazebo::ModelPlugin
    {

    public:

        /** Constructor
         * 
         * Create subscriber to gazebo and publishers to publish
         * thruster force values to gazebo
         * 
         */
        ThrusterDriver(void);

        /** Destructor
         * 
         */
        ~ThrusterDriver(void);

        /** Collects all neccessary parameters and initializes the ROS 2 node.
         * 
         * @param _model A pointer to the attached mdoel
         * @param _sdf   A pointer to the robot's SDF description
         */
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:

        /** Receives the force values as a vector from the thrust allocation node
         * 
         * Called continuously and pass each force value to the correct thruster.
         * 
         * @param joint_cmd command from gazebo containing the thruster forces
         */
        void GetForceCmd(const std_msgs::msg::Float64MultiArray::SharedPtr joint_cmd);

        /** Publishes a fixed force value to each thruster in gazebo
         * 
         * Updates in sequence with gazebo's main world update function.
         * 
         */
        void ApplyForce(void);

        /** Allows the node to run with Gazebo 
         * 
         * Blocking function that spins on a dedicated thread. 
         * 
         */
        void SpinNode(void);

        int thruster_count;
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_cmd;
        std::vector<gazebo::physics::LinkPtr> thruster;
        std::vector<double> thrust_values;
        gazebo::event::ConnectionPtr updateConnection_;

        std::thread spinThread;
        std::string topic_name;

        // void SpinNode(void);
        // void GetForceCmd(const std_msgs::msg::Float64MultiArray::SharedPtr);
        // void ApplyForce(void);
    // public: 
    //     ThrusterDriver(void);
    //     ~ThrusterDriver(void);
    //     virtual void Load(gazebo::physics::ModelPtr, sdf::ElementPtr);
    };

    GZ_REGISTER_MODEL_PLUGIN(ThrusterDriver)

}
#endif // TRITON_GAZEBO__THRUSTER_DRIVER_PLUGIN