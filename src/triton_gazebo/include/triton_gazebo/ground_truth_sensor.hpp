#ifndef TRITON_GAZEBO__GROUND_TRUTH_SENSOR
#define TRITON_GAZEBO__GROUND_TRUTH_SENSOR

#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace triton_gazebo
{

    using std::placeholders::_1;

    class GroundTruthSensor : public gazebo::ModelPlugin
    {

    public:

        // Constructor
        GroundTruthSensor(void);

        // Destructor
        ~GroundTruthSensor(void);

        /** Collects all neccessary parameters and initializes the ROS 2 node.
         * 
         * @param _model A pointer to the attached mdoel
         * @param _sdf   A pointer to the robot's SDF description
         */
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /** Publishes the ground truth pose of the AUV
         * 
         */
        virtual void OnUpdate();

    private:

        /** Spins ROS2 node on a dedicated thread to remain non-blocking
         * 
         */
        void SpinNode(void);

        rclcpp::Node::SharedPtr node;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher;
        gazebo::event::ConnectionPtr updateConnection_;
        gazebo::physics::ModelPtr model;
        std::string state_topic;
        int count;

        std::thread spinThread;
        std::string topic_name;
    };

    GZ_REGISTER_MODEL_PLUGIN(GroundTruthSensor)

}
#endif // TRITON_GAZEBO__GROUND_TRUTH_SENSOR
