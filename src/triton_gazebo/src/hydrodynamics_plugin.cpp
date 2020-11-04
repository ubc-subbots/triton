#include "triton_gazebo/hydrodynamics_plugin.hpp"

namespace triton_gazebo
{
    HydrodynamicsPlugin::HydrodynamicsPlugin (){}

    HydrodynamicsPlugin::~HydrodynamicsPlugin (){}

    void HydrodynamicsPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        model_ = model;
        updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&HydrodynamicsPlugin::OnUpdate, this));

    }

    void HydrodynamicsPlugin::OnUpdate()
    {
        model_->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }
    
} // namespace triton_gazebo
