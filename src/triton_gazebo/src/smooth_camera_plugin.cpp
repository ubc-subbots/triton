#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "triton_gazebo/smooth_camera_plugin.hpp"
#include <random>
#include <cmath>
#include <time.h>

using namespace gazebo;

namespace triton_gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(triton_gazebo::SmoothCameraPlugin)
    SmoothCameraPlugin::SmoothCameraPlugin(){}

    SmoothCameraPlugin::~SmoothCameraPlugin(){}

    void SmoothCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        GZ_ASSERT(_model, "Model pointer is null");
        this->camera_model_ = _model;

        // Make sure tracked model has been specified
        if (!_sdf->HasElement("track_model"))
        {
            gzerr << "<track_model> element missing from SmoothCamera plugin. "
            << "The plugin will not function.\n";
            return;
        }
        this->track_model_ = camera_model_->GetWorld()->ModelByName(_sdf->GetElement("track_model")->Get<std::string>());
        
        // Make sure spin_speed has been specified
        if (!_sdf->HasElement("spin_speed"))
        {
            gzerr << "<spin_speed> element missing from SmoothCamera plugin. "
            << "The plugin will not function.\n";
            return;
        }
        this->spin_speed = _sdf->GetElement("spin_speed")->Get<double>();

        // Get ranges for random parameters
        if (_sdf->HasElement("radius_range"))
            radius_range_ = _sdf->Get<ignition::math::Vector2d>("radius_range");
        else
            radius_range_ = {1,5};

        // Connect to the world update signal
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SmoothCameraPlugin::Update, this, std::placeholders::_1));

        rotw = -M_PI;
        rotx = 0;
        roty = 0;
        rotz = 1;
        radius = radius_range_[0];
        theta = M_PI/2;
        phi = 0;
        incr_phi = 1;
        axis_theta = -M_PI;
        spinned_one_cycle = 0;
        change_pos = 0;
    }


    void SmoothCameraPlugin::Reset()
    {
        ignition::math::Vector3d camera_pos;
        /*
         * axis_theta goes from -M_PI to M_PI
         * For each value of axis_theta, the camera is rotated 360 degrees gradually
         * Everytime axis_theta reaches M_PI, it resets and the position of the camera is changed.
         */
        if (!spinned_one_cycle) {
            if (rotw < M_PI) {
                rotw += 0.0001;
            }
            else {
                spinned_one_cycle = 1;
                rotw = -M_PI;
            }
        }
        else {
            spinned_one_cycle = 0;
            if (axis_theta < M_PI) {
                axis_theta += 0.01;
                rotz = cos(axis_theta);
                roty = sin(axis_theta);
            }
            else {
                axis_theta = -M_PI;
                rotz = 1;
                roty = 0;
                change_pos = 1;
            }
        }
        /* 
         * theta is the angle from the z-axis, and phi is the angle from the x-axis
         * incr_phi is used to change the direction of which the camera rotates about the z-axis
         * This prevents the camera from 'teleporting' to a completely new position 
         * and causing timing issues that affects data generation.
         */
        if (change_pos) {
            change_pos = 0;
            if ((phi < M_PI / 2 && incr_phi) || (phi > 0 && !incr_phi)) {
                phi += double(incr_phi) * spin_speed;
            }
            else if (theta > 0) {
                theta -= spin_speed;
                incr_phi = incr_phi ? 0 : 1;
            }
            else {
                /* At the end, we reset everything but with an increased radius */
                rotw = -M_PI;
                rotx = 0;
                roty = 0;
                rotz = 1;
                theta = M_PI/2;
                phi = 0;
                incr_phi = 1;
                axis_theta = -M_PI;
                spinned_one_cycle = 0;
                change_pos = 0;

                if (radius < radius_range_[1]) {
                    radius += 1;
                }
                else {
                    change_pos = 1;
                    theta = -1;
                    phi = -1;
                    incr_phi = 0;
                    std::cout << "END ME PLEASE" << std::endl;
                    return;
                }
            }
            camera_pos.X(radius * sin(theta) * cos(phi));
            camera_pos.Y(radius * sin(theta) * sin(phi));
            camera_pos.Z(radius * cos(theta));
        }
        else {
            camera_pos = camera_model_->WorldPose().Pos();
        }


        ignition::math::Quaterniond camera_rot(rotw, rotx, roty, rotz);

        ignition::math::Pose3d camera_pose;
        camera_pose.Set(camera_pos,camera_rot);
        camera_model_->SetWorldPose(camera_pose);
    }
    void SmoothCameraPlugin::Update(const common::UpdateInfo &_info)
    {
        // // Reinitialize camera and orbit with new parameters when enough time has elapsed
        // if (_info.simTime - prev_update_ > period_)
        // {
        //     prev_update_ = _info.simTime;
            Reset();
        //}
    }
}

