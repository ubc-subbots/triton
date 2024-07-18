#ifndef TRITON_GAZEBO__CAMERA_ORBIT_PLUGIN
#define TRITON_GAZEBO__CAMERA_ORBIT_PLUGIN

#include <sdf/sdf.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>

namespace triton_gazebo
{
  class GAZEBO_VISIBLE CameraOrbitPlugin : public gazebo::ModelPlugin
  {
    public:
    CameraOrbitPlugin();
    ~CameraOrbitPlugin();

    virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Reset();

    private: 
    /** Update the plugin once every iteration of simulation.
     * @param _info World update information
     */
    void Update(const gazebo::common::UpdateInfo &_info);

    //double radius_;
    double period_ = 10;
    double orbit_angle_x_ = 0;
    double orbit_angle_y_ = 0;
    double camera_angle_x_ = 0;
    double camera_angle_y_ = 0;
    double camera_angle_z_ = 0;
    gazebo::physics::ModelPtr track_model_;
    gazebo::physics::ModelPtr camera_model_;
    gazebo::physics::LinkPtr link_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::common::Time prev_update_;

    ignition::math::Vector2d radius_range_;
    ignition::math::Vector2d height_range_;
    ignition::math::Vector2d orbit_angle_range_;
    ignition::math::Vector2d camera_angle_range_;
  };
}
#endif