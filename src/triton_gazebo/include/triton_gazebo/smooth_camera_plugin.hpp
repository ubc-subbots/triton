#ifndef TRITON_GAZEBO__SMOOTH_CAMERA_PLUGIN
#define TRITON_GAZEBO__SMOOTH_CAMERA_PLUGIN

#include <sdf/sdf.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>

namespace triton_gazebo
{
  class GAZEBO_VISIBLE SmoothCameraPlugin : public gazebo::ModelPlugin
  {
    public:
    SmoothCameraPlugin();
    ~SmoothCameraPlugin();

    virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Reset();

    private: 
    /** Update the plugin once every iteration of simulation.
     * @param _info World update information
     */
    void Update(const gazebo::common::UpdateInfo &_info);

    gazebo::physics::ModelPtr track_model_;
    gazebo::physics::ModelPtr camera_model_;
    gazebo::physics::LinkPtr link_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::common::Time prev_update_;

    ignition::math::Vector2d radius_range_;


    double rotw, rotx, roty, rotz;
    // For positioning the camera
    double theta, phi;
    int incr_phi;
    // For adjusting rotational axis
    double axis_theta;
    // For keeping track of rotation and movement
    int spinned_one_cycle;
    int change_pos;
    // For adjusting distance from object
    double radius;
  };
}
#endif