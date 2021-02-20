#include "triton_gazebo/hydrodynamics_plugin.hpp"

namespace triton_gazebo
{
    HydrodynamicsPlugin::HydrodynamicsPlugin() {}

    HydrodynamicsPlugin::~HydrodynamicsPlugin() {}

    void HydrodynamicsPlugin::Load(gazebo::physics::ModelPtr _model, 
                                   sdf::ElementPtr _sdf)
    {
        model = _model;
        frame = model->GetLink("frame::frame");
        this->mass = frame->GetInertial()->Mass();

        this->fluid_density = 1028.00;
        this->gravity = 9.81;

        this->Connect();
    }

    void HydrodynamicsPlugin::Connect()
    {
        updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&HydrodynamicsPlugin::Update, this, std::placeholders::_1));
    }

    void HydrodynamicsPlugin::Update(const gazebo::common::UpdateInfo &_info)
    {
        Eigen::Vector6d velocity = this->GetVelocityVector();
        Eigen::Vector6d acceleration = this->GetAccelerationVector();

        SetWrenchVector(-this->mass * acceleration);
    }

    Eigen::Vector6d HydrodynamicsPlugin::GetVelocityVector()
    {
        ignition::math::Vector3d linear_vel = this->frame->RelativeLinearVel();
        ignition::math::Vector3d angular_vel = this->frame->RelativeAngularVel();
        Eigen::Vector6d velocity;

        // Convert velocity into Eigen library vector
        velocity << linear_vel.X(),
                    linear_vel.Y(),
                    linear_vel.Z(),
                    angular_vel.X(),
                    angular_vel.Y(),
                    angular_vel.Z();

        return velocity;
    }

    Eigen::Vector6d HydrodynamicsPlugin::GetAccelerationVector()
    {
        ignition::math::Vector3d linear_acc = this->frame->RelativeLinearAccel();
        ignition::math::Vector3d angular_acc = this->frame->RelativeAngularAccel();
        Eigen::Vector6d acceleration;

        // Convert acceleration into Eigen library vector
        acceleration << linear_acc.X(),
                        linear_acc.Y(),
                        linear_acc.Z(),
                        angular_acc.X(),
                        angular_acc.Y(),
                        angular_acc.Z();

        return acceleration;
    }

    void HydrodynamicsPlugin::SetWrenchVector(Eigen::Vector6d wrench)
    {
        ignition::math::Vector3d force(wrench[0], wrench[1], wrench[2]);
        ignition::math::Vector3d torque(wrench[3], wrench[4], wrench[5]);

/*      This sequence should cover hydrodynamic forces
        Eigen::Vector6d velRel, acc;
        // Compute the relative velocity
        velRel = EigenStack(
        this->ToNED(linVel - flowVel),
        this->ToNED(angVel));

        // Update added Coriolis matrix
        this->ComputeAddedCoriolisMatrix(velRel, this->Ma, this->Ca);

        // Update damping matrix
        this->ComputeDampingMatrix(velRel, this->D);

        // Filter acceleration (see issue explanation above)
        this->ComputeAcc(velRel, _time, 0.3);

        // We can now compute the additional forces/torques due to thisdynamic
        // effects based on Eq. 8.136 on p.222 of Fossen: Handbook of Marine Craft ...

        // Damping forces and torques
        Eigen::Vector6d damping = -this->D * velRel;

        // Added-mass forces and torques
        Eigen::Vector6d added = -this->GetAddedMass() * this->filteredAcc;

        // Added Coriolis term
        Eigen::Vector6d cor = -this->Ca * velRel;

        // All additional (compared to standard rigid body) Fossen terms combined.
        Eigen::Vector6d tau = damping + added + cor;

        if (!std::isnan(tau.norm()))
        {
            // Convert the forces and moments back to Gazebo's reference frame
            ignition::math::Vector3d hydForce =
                this->FromNED(Vec3dToGazebo(tau.head<3>()));
            ignition::math::Vector3d hydTorque =
                this->FromNED(Vec3dToGazebo(tau.tail<3>()));

            // Forces and torques are also wrt link frame
            this->link->AddRelativeForce(hydForce);
            this->link->AddRelativeTorque(hydTorque);
        }

        this->ApplyBuoyancyForce();
*/


        frame->AddForce(force);
        frame->AddTorque(torque);
    }

    void HydrodynamicsPlugin::ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel, const Eigen::Matrix6d& _Ma, Eigen::Matrix6d &_Ca) const
    {

    }

    void HydrodynamicsPlugin::ComputeDampingMatrix(const Eigen::Vector6d& _vel, Eigen::Matrix6d &_D) const
    {

    }

    Eigen::Matrix6d HydrodynamicsPlugin::GetAddedMass() const
    {
        Eigen::Matrix6d M_d;
        
        return M_d;
    }

} // namespace triton_gazebo
