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
        if (_sdf->HasElement("update_rate"))
        {
            this->update_rate = _sdf->Get<int>("update_rate");
        }
        else
        {
            gzmsg << "update_rate value not specified, using default: 1Hz.\n";
            this->update_rate = 1;
        }

        this->state_publisher = node->
            create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(this->state_topic, 10);

        this->model = _model;
    
        this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                                    std::bind(&GroundTruthSensor::OnUpdate, this));

        /// @todo feels like there should be a way to pass rclcpp::spin directly to thread, this works the way it is though 
        this->spinThread = std::thread(std::bind(&GroundTruthSensor::SpinNode, this));

        this->prev_time = node->now();

        gzmsg << "Ground Truth sensor successfully started!\n";
    }

    void GroundTruthSensor::OnUpdate()
    {
        ignition::math::Pose3d pose = model->WorldPose();
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.pose.pose.position.x = pose.Pos()[0];
        msg.pose.pose.position.y = pose.Pos()[1];
        msg.pose.pose.position.z = pose.Pos()[2];
        msg.pose.pose.orientation.x = pose.Rot().X();
        msg.pose.pose.orientation.y = pose.Rot().Y();
        msg.pose.pose.orientation.z = pose.Rot().Z();
        msg.pose.pose.orientation.w = pose.Rot().W();
        // TODO: set a reasonable covariance
        msg.pose.covariance[0] = 0.001;
        msg.pose.covariance[7] = 0.001;
        msg.pose.covariance[14] = 0.001;
        msg.pose.covariance[21] = 0.001;
        msg.pose.covariance[28] = 0.001;
        msg.pose.covariance[35] = 0.001;
        rclcpp::Time now = node->now();
        msg.header.stamp = now;
        msg.header.frame_id = "map";
        if ((now- this->prev_time).seconds() >= (1.0/this->update_rate))
        {
            this->prev_time = now;
            this->state_publisher->publish(msg);
        }
    }

    void GroundTruthSensor::SpinNode()
    {
        rclcpp::spin(node);
    }

}
