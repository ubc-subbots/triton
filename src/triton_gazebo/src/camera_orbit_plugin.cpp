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
        if (_sdf->HasElement("angle"))
            angle_ = _sdf->Get<double>("angle");

        // Connect to the world update signal
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CameraOrbitPlugin::Update, this, std::placeholders::_1));
    }

    void CameraOrbitPlugin::Reset(){}

    void CameraOrbitPlugin::Update(const common::UpdateInfo &_info)
    {
        // Calculate displacement vector from tracked object to camera
        ignition::math::Vector3d r = camera_model_->WorldPose().Pos() - track_model_->WorldPose().Pos();

        // Calculate speed required for stable orbit
        double speed = r.Length()*ignition::math::Angle::TwoPi()/period_;

        // Calculate up vector of orbit
        ignition::math::Quaterniond orbit_rot;
        orbit_rot.Axis(ignition::math::Vector3d::UnitY,angle_);
        //ignition::math::Vector3d track_up = track_model_->WorldPose().Rot() * ignition::math::Vector3d::UnitZ;
        ignition::math::Vector3d track_up = orbit_rot * ignition::math::Vector3d::UnitZ;

        // Calculate direction of velocity (tangent to orbit circle)
        ignition::math::Vector3d v_hat = track_up.Cross(r).Normalized();

        // Calculate new velocity of camera, plus the tracked object's velocity
        ignition::math::Vector3d v = v_hat * speed + track_model_->WorldLinearVel();

        // Calculate quaternion for camera looking at tracked object
        ignition::math::Quaterniond camera_rot;
        camera_rot.From2Axes(ignition::math::Vector3d::UnitX,-r.Normalized());

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

