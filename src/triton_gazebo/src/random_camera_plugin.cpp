#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "triton_gazebo/random_camera_plugin.hpp"
#include <random>
#include <cmath>
#include <time.h>

using namespace gazebo;

namespace triton_gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(triton_gazebo::RandomCameraPlugin)
    RandomCameraPlugin::RandomCameraPlugin(){}

    RandomCameraPlugin::~RandomCameraPlugin(){}

    void RandomCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        GZ_ASSERT(_model, "Model pointer is null");
        this->camera_model_ = _model;

        // Make sure tracked model has been specified
        if (!_sdf->HasElement("track_model"))
        {
            gzerr << "<track_model> element missing from RandomCamera plugin. "
            << "The plugin will not function.\n";
            return;
        }
        this->track_model_ = camera_model_->GetWorld()->ModelByName(_sdf->GetElement("track_model")->Get<std::string>());

          // Get ranges for random parameters; Resets after each period
        if (_sdf->HasElement("radius_range"))
            radius_range_ = _sdf->Get<ignition::math::Vector2d>("radius_range");
        else
            radius_range_ = {2,20};
        
        // Connect to the world update signal
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&RandomCameraPlugin::Update, this, std::placeholders::_1));
    }

    void RandomCameraPlugin::Reset()
    {
        // Random number generator
        std::uniform_real_distribution<double> unif(0,1);
        std::default_random_engine re(time(NULL));

        // Set random radius offset from tracked model's position
        double radius = unif(re) * (radius_range_.Y()-radius_range_.X()) + radius_range_.X();

        // Generate random quaternion and move camera in that direction by radius
        ignition::math::Quaterniond radial;
        double u1 = unif(re);
        double u2 = unif(re);
        double u3 = unif(re);
        radial.Set(sqrt(1-u1)*sin(2*M_PI*u2), sqrt(1-u1)*cos(2*M_PI*u2), sqrt(u1)*sin(2*M_PI*u3), sqrt(u1)*cos(2*M_PI*u3));

        ignition::math::Vector3d camera_pos = ignition::math::Vector3d::UnitZ * radius;
        camera_pos = radial * camera_pos;
        camera_pos = camera_pos + track_model_->WorldPose().Pos();

        // Generate random quaternion for camera rotation
        ignition::math::Quaterniond camera_rot;
        u1 = unif(re);
        u2 = unif(re);
        u3 = unif(re);
        camera_rot.Set(sqrt(1-u1)*sin(2*M_PI*u2), sqrt(1-u1)*cos(2*M_PI*u2), sqrt(u1)*sin(2*M_PI*u3), sqrt(u1)*cos(2*M_PI*u3));

        // Set new camera pose
        ignition::math::Pose3d camera_pose;
        camera_pose.Set(camera_pos,camera_rot);
        camera_model_->SetWorldPose(camera_pose);
    }

    void RandomCameraPlugin::Update(const common::UpdateInfo &_info)
    {
        // // Reinitialize camera and orbit with new parameters when enough time has elapsed
        // if (_info.simTime - prev_update_ > period_)
        // {
        //     prev_update_ = _info.simTime;
            Reset();
        //}
    }
}
