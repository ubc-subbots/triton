#ifndef TRITON_GAZEBO__BOUNDING_BOX_PLUGIN
#define TRITON_GAZEBO__BOUNDING_BOX_PLUGIN

#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/CameraSensor.hh>

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
    std::vector<ignition::math::Vector3d> corners_;

  };
}
GZ_REGISTER_SENSOR_PLUGIN(triton_gazebo::BoundingBoxPlugin)
#endif