#include "triton_gazebo/hydrodynamics_plugin.hpp"

namespace triton_gazebo
{
    HydrodynamicsPlugin::HydrodynamicsPlugin()
    {
        velocity = Eigen::VectorXd(6);
        acceleration = Eigen::VectorXd(6);
    }

    HydrodynamicsPlugin::~HydrodynamicsPlugin() {}

    void HydrodynamicsPlugin::Load(gazebo::physics::ModelPtr _model, 
                                   sdf::ElementPtr _sdf)
    {
        model = _model;
        frame = model->GetLink("frame::frame");
        gzerr << std::endl << frame->GetName() << std::endl;

        /*
        links = model->GetLinks();
        for (int i = 0; i < (int)links.size(); i++)
        {
            gzerr << std::endl << links[i]->GetName() << std::endl;
        }
        */

        this->Connect();
    }

    void HydrodynamicsPlugin::Connect()
    {
        updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&HydrodynamicsPlugin::Update, this, std::placeholders::_1));
    }

    void HydrodynamicsPlugin::Update(const gazebo::common::UpdateInfo &_info)
    {
        this->GetVelocityVector();
        frame->	AddForce(ignition::math::Vector3d(100, 0, 0));
    }

    void HydrodynamicsPlugin::GetVelocityVector()
    {
        ignition::math::Vector3d linear_vel = this->frame->RelativeLinearVel();
        ignition::math::Vector3d angular_vel = this->frame->RelativeAngularVel();

        // Convert velocity into Eigen library vector
        this->velocity << linear_vel.X(),
                          linear_vel.Y(),
                          linear_vel.Z(),
                          angular_vel.X(),
                          angular_vel.Y(),
                          angular_vel.Z();

        //std::cout << this->velocity << std::endl;
    }

    /// @todo According to Plankton comments, Gazebo may report bad values, 
    ///       not sure if this is still the case
    void HydrodynamicsPlugin::GetAccelerationVector()
    {
        ignition::math::Vector3d linear_acc = this->frame->RelativeLinearAccel();
        ignition::math::Vector3d angular_acc = this->frame->RelativeAngularAccel();

        // Convert acceleration into Eigen library vector
        this->acceleration << linear_acc.X(),
                              linear_acc.Y(),
                              linear_acc.Z(),
                              angular_acc.X(),
                              angular_acc.Y(),
                              angular_acc.Z();

        //std::cout << this->acceleration << std::endl;
    }

    /// @brief Adapted from Plankton simulation
    void ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel,
                                            const Eigen::Matrix6d& _Ma,
                                            Eigen::Matrix6d &_Ca) const
    {
        // This corresponds to eq. 6.43 on p. 120 in
        // Fossen, Thor, "Handbook of Marine Craft and Hydrodynamics and Motion
        // Control", 2011
        Eigen::Vector6d ab = this->GetAddedMass() * _vel;
        Eigen::Matrix3d Sa = -1 * CrossProductOperator(ab.head<3>());
        _Ca << Eigen::Matrix3d::Zero(), Sa,
                Sa, -1 * CrossProductOperator(ab.tail<3>());
    }
-
    void ComputeDampingMatrix(const Eigen::Vector6d& _vel,
                                        Eigen::Matrix6d &_D) const
    {
        // From Antonelli 2014: the viscosity of the fluid causes
        // the presence of dissipative drag and lift forces on the
        // body. A common simplification is to consider only linear
        // and quadratic damping terms and group these terms in a
        // matrix Drb

        _D.setZero();

        _D = -1 *
            (this->DLin + this->offsetLinearDamping * Eigen::Matrix6d::Identity()) -
            _vel[0] * (this->DLinForwardSpeed +
            this->offsetLinForwardSpeedDamping * Eigen::Matrix6d::Identity());

        // Nonlinear damping matrix is considered as a diagonal matrix
        for (int i = 0; i < 6; i++)
        {
            _D(i, i) += -1 *
            (this->DNonLin(i, i) + this->offsetNonLinDamping) *
            std::fabs(_vel[i]);
        }
        _D *= this->scalingDamping;
    }

    /////////////////////////////////////////////////
    Eigen::Matrix6d GetAddedMass() const
    {
        return this->scalingAddedMass *
            (this->Ma + this->offsetAddedMass * Eigen::Matrix6d::Identity());
    }

} // namespace triton_gazebo
