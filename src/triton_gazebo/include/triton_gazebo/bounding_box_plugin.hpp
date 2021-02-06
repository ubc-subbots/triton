#ifndef TRITON_GAZEBO__BOUNDING_BOX_PLUGIN
#define TRITON_GAZEBO__BOUNDING_BOX_PLUGIN

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include "rclcpp/rclcpp.hpp"
#include "triton_interfaces/msg/detection_box.hpp"

namespace triton_gazebo
{
  class BoundingBoxPlugin : public gazebo::SensorPlugin
  {
    public: 
    
    BoundingBoxPlugin();
    ~BoundingBoxPlugin();

    virtual void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:

    virtual void OnUpdate();
    gazebo::sensors::CameraSensorPtr parentSensor;
    gazebo::event::ConnectionPtr updateConnection;
    std::string model_name_;

    //ROS
    rclcpp::Node * node_;
    rclcpp::Publisher<triton_interfaces::msg::DetectionBox>::SharedPtr publisher_; 

    static int powint(int x, int p)//https://stackoverflow.com/a/1505791
    {
        if (p == 0) return 1;
        if (p == 1) return x;
        int tmp = powint(x, p/2);
        if (p%2 == 0) return tmp * tmp;
        else return x * tmp * tmp;
    }

  };
}
GZ_REGISTER_SENSOR_PLUGIN(triton_gazebo::BoundingBoxPlugin)
#endif