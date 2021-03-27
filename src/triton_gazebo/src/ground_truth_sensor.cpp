#include "triton_gazebo/ground_truth_sensor.hpp"
#include <ignition/math/Pose3.hh>

namespace triton_gazebo
{

    GroundTruthSensor::GroundTruthSensor() : node{rclcpp::Node::make_shared("ground_truth_sensor")} {}


    GroundTruthSensor::~GroundTruthSensor() {}


    void GroundTruthSensor::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        if (_sdf->HasElement("state_topic"))
        {
            this->state_topic = _sdf->Get<std::string>("state_topic");
        }
        else
        {
            gzerr << "state_topic value not specified, exiting.\n";
            exit(1);
        }

        this->state_publisher = node->
            create_publisher<geometry_msgs::msg::PoseStamped>(this->state_topic, 10);

        this->model = _model;
    
        this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                                    std::bind(&GroundTruthSensor::OnUpdate, this));

        /// @todo feels like there should be a way to pass rclcpp::spin directly to thread, this works the way it is though 
        this->spinThread = std::thread(std::bind(&GroundTruthSensor::SpinNode, this));
    }

    void GroundTruthSensor::OnUpdate()
    {
        ignition::math::Pose3d pose = model->WorldPose();
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.pose.position.x = pose.Pos()[0];
        msg.pose.position.y = pose.Pos()[1];
        msg.pose.position.z = pose.Pos()[2];
        msg.pose.orientation.x = pose.Rot().X();
        msg.pose.orientation.y = pose.Rot().Y();
        msg.pose.orientation.z = pose.Rot().Z();
        msg.pose.orientation.w = pose.Rot().W();
        msg.header.stamp = node->now();
        msg.header.frame_id = "odom";
        if (count % 10 == 0) // send every 10 updates, should instead send on a given frequency
        {
            state_publisher->publish(msg);
        }
        count++;
    }

    void GroundTruthSensor::SpinNode()
    {
        rclcpp::spin(node);
    }

}
