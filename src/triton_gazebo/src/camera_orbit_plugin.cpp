#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "triton_gazebo/camera_orbit_plugin.hpp"

using namespace gazebo;

namespace triton_gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(triton_gazebo::CameraOrbitPlugin)
    CameraOrbitPlugin::CameraOrbitPlugin(){}

    CameraOrbitPlugin::~CameraOrbitPlugin(){}

    void CameraOrbitPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        GZ_ASSERT(_model, "Model pointer is null");
        this->camera_model_ = _model;

        // Make sure tracked model has been specified
        if (!_sdf->HasElement("track_model"))
        {
            gzerr << "<track_model> element missing from CameraOrbit plugin. "
            << "The plugin will not function.\n";
            return;
        }
        this->track_model_ = camera_model_->GetWorld()->ModelByName(_sdf->GetElement("track_model")->Get<std::string>());

        // Get parameters
        if (_sdf->HasElement("period"))
            period_ = _sdf->Get<double>("period");
        if (_sdf->HasElement("orbit_angle_x"))
            orbit_angle_x_ = _sdf->Get<double>("orbit_angle_x");
        if (_sdf->HasElement("orbit_angle_y"))
            orbit_angle_y_ = _sdf->Get<double>("orbit_angle_y");
        if (_sdf->HasElement("camera_angle_x"))
            camera_angle_x_ = _sdf->Get<double>("camera_angle_x");
        if (_sdf->HasElement("camera_angle_y"))
            camera_angle_y_ = _sdf->Get<double>("camera_angle_y");
        if (_sdf->HasElement("camera_angle_z"))
            camera_angle_z_ = _sdf->Get<double>("camera_angle_z");

        // Connect to the world update signal
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CameraOrbitPlugin::Update, this, std::placeholders::_1));
    }

    void CameraOrbitPlugin::Reset(){}

    void CameraOrbitPlugin::Update(const common::UpdateInfo &_info)
    {
        // Calculate displacement vector from tracked object to camera
        ignition::math::Vector3d d = camera_model_->WorldPose().Pos() - track_model_->WorldPose().Pos();

        // Calculate up vector of orbit; we start with unit-z and rotate about the x-axis and y-axis to tilt the orbit axis
        ignition::math::Quaterniond orbit_rot_x;
        orbit_rot_x.Axis(ignition::math::Vector3d::UnitX,orbit_angle_x_);
        ignition::math::Quaterniond orbit_rot_y;
        orbit_rot_y.Axis(ignition::math::Vector3d::UnitY,orbit_angle_y_);
        ignition::math::Quaterniond orbit_rot = orbit_rot_x*orbit_rot_y;
        ignition::math::Vector3d orbit_up = orbit_rot * ignition::math::Vector3d::UnitZ;

        // Calculate direction of velocity (tangent to orbit circle)
        ignition::math::Vector3d v_hat = orbit_up.Cross(d).Normalized();

        // Calculate radial direction of orbit circle
        ignition::math::Vector3d r_hat = v_hat.Cross(orbit_up).Normalized();

        // Calculate speed required for stable orbit
        double speed = r_hat.Dot(d)*ignition::math::Angle::TwoPi()/period_;

        // Calculate new velocity of camera, plus the tracked object's velocity
        ignition::math::Vector3d v = v_hat * speed + track_model_->WorldLinearVel();

        // Calculate quaternion for camera; we start with -r_hat (looking toward orbit axis) and rotate about x,y,z axes (in camera coordinate frame)
        ignition::math::Quaterniond camera_rot_x;
        camera_rot_x.Axis(ignition::math::Vector3d::UnitX,camera_angle_x_);
        ignition::math::Quaterniond camera_rot_y;
        camera_rot_y.Axis(ignition::math::Vector3d::UnitY,camera_angle_y_);
        ignition::math::Quaterniond camera_rot_z;
        camera_rot_z.Axis(ignition::math::Vector3d::UnitZ,camera_angle_z_);
        ignition::math::Quaterniond camera_rot;
        camera_rot.From2Axes(ignition::math::Vector3d::UnitX,-r_hat);
        camera_rot = camera_rot*camera_rot_x*camera_rot_y*camera_rot_z;

        // Set new camera pose
        ignition::math::Pose3d camera_pose;
        camera_pose.Set(camera_model_->WorldPose().Pos(),camera_rot);
        camera_model_->SetWorldPose(camera_pose);

        // Apply velocity to camera
        camera_model_->SetLinearVel(v);

        // // Change direction when enough time has elapsed
        // if (_info.simTime - this->dataPtr->prevUpdate > this->dataPtr->updatePeriod)
        // {
        //     this->dataPtr->prevUpdate = _info.simTime;
        // }
    }
}

