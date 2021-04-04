#include "triton_gazebo/hydrodynamics_plugin.hpp"

namespace triton_gazebo
{

    HydrodynamicsPlugin::HydrodynamicsPlugin() {}


    HydrodynamicsPlugin::~HydrodynamicsPlugin() {}


    void HydrodynamicsPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model = _model;

        // Get params that apply to the simulation world
        GetWorldParameters(this->model->GetWorld()->SDF());

        // Get params for model dynamics and hydro model
        GetModelParameters(this->model->GetSDF());

        this->mass = frame->GetInertial()->Mass();

        this->DLinForwardSpeed.setZero();

        this->Connect();
    }

    // Return a status flag if needed later on
    bool HydrodynamicsPlugin::GetWorldParameters(sdf::ElementPtr world_sdf)
    {
        bool status = true;
        Eigen::Vector6d gravity_eig;
    
        this->fluid_density = GetSdfElement<double>(&status, world_sdf, "fluid_density", 1028.00);

        gravity_eig << 0.00, 0.00, -9.81, 0.00, 0.00, 0.00;
        gravity_eig = GetSdfVector(&status, world_sdf, "gravity", gravity_eig);

        this->gravity = ignition::math::Vector3d(gravity_eig[0], gravity_eig[1], gravity_eig[2]);

        return status;
    }


    bool HydrodynamicsPlugin::GetModelParameters(sdf::ElementPtr model_sdf)
    {
        sdf::ElementPtr hydro_model = model_sdf->GetElement("hydrodynamic_model");
        bool status = true;

        std::string base_link = GetSdfElement<std::string>(&status, model_sdf, "base_link", "");
        if (!status)
        {
            gzerr << "Base link not specified! Exiting.\n";
            exit(1);
        }
        this->frame = model->GetLink(base_link);

        // Added mass manipulators
        this->scalingAddedMass = GetSdfElement<double>(&status, hydro_model, "scalingAddedMass", 1.00);
        this->offsetAddedMass = GetSdfElement<double>(&status, hydro_model, "offsetAddedMass", 0.00);

        // Added mass parameters, assume zero if not specified 
        this->added_mass = GetSdfMatrix(&status, hydro_model, "added_mass");

        // Linear damping manipulators 
        this->scalingDamping = GetSdfElement<double>(&status, hydro_model, "scalingDamping", 1.00);
        this->offsetLinearDamping = GetSdfElement<double>(&status, hydro_model, "offetLineaDamping", 0.00);
        this->offsetLinForwardSpeedDamping = GetSdfElement<double>(&status, hydro_model, "offsetLinForwardSpeedDamping", 0.00);

        // Non-linear damping manipulators 
        this->offsetNonLinDamping = GetSdfElement<double>(&status, hydro_model, "offsetNonLinDamping", 0.00);

        this->volume = GetSdfElement<double>(&status, model_sdf, "volume");
        if (!status)
        {
            gzerr << "Model volume not specified.\n";
            exit(1);
        }

        // Make no assumptions for CoB
        Eigen::Vector6d CoB_eig = GetSdfVector(&status, model_sdf, "center_of_buoyancy");
        if (!status)
        {
            gzerr << "Center of Buoyancy not specified.\n";
            exit(1);
        }
        // Calculate CoB relative to CoM rather than model origin
        ignition::math::Vector3d CoM = frame->GetInertial()->Pose().Pos();
        this->rel_CoB = ignition::math::Vector3d(CoB_eig[0], CoB_eig[1], CoB_eig[2]) - CoM;

        // For now we will approximate linear damping as a diagnol matrix
        Eigen::Vector6d linear_damping_vec = GetSdfVector(&status, hydro_model, "linear_damping");
        this->linear_damping = ToDiagonalMatrix(linear_damping_vec);

        // For now we will approximate non-linear damping as a diagnol matrix
        this->quadratic_damping = GetSdfVector(&status, hydro_model, "quadratic_damping");
        this->non_linear_damping = ToDiagonalMatrix(quadratic_damping);

        this->scalingBuoyancy = GetSdfElement<double>(&status, hydro_model, "scalingBuoyancy", 1.00);
        this->is_neutrally_buoyant = GetSdfElement<bool>(&status, model_sdf, "neutrally_buoyant", false);

        return status;
    }


    void HydrodynamicsPlugin::Connect()
    {
        updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&HydrodynamicsPlugin::Update, this));
    }


    void HydrodynamicsPlugin::Update()
    {
        Eigen::Vector6d velocity = this->GetVelocityVector();
        Eigen::Vector6d acceleration = this->GetAccelerationVector();
        Eigen::Vector6d velRel = ToNED(velocity);

        // Update added Coriolis matrix
        this->ComputeAddedCoriolisMatrix(velRel, this->added_mass, this->coriolis_matrix);

        // Update damping matrix
        this->ComputeDampingMatrix(velRel, this->damping_matrix);

        Eigen::Matrix6d Ma = this->GetAddedMass();

        // We can now compute the additional forces/torques due to thisdynamic
        // effects based on Eq. 8.136 on p.222 of Fossen: Handbook of Marine Craft ...

        // Damping forces and torques
        Eigen::Vector6d damping = -this->damping_matrix * velRel;

        // Added-mass forces and torques
        Eigen::Vector6d added = -Ma * acceleration;

        // Added Coriolis term
        Eigen::Vector6d cor = -this->coriolis_matrix * velRel;

        // All additional (compared to standard rigid body) Fossen terms combined.
        Eigen::Vector6d tau = damping + added + cor;

        GZ_ASSERT(!std::isnan(tau.norm()), "Hydrodynamic forces vector is nan");

        if (!std::isnan(tau.norm()))
        {
            this->SetWrenchVector(FromNED(tau));
        }

        this->ApplyBuoyancyForce();
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
        ignition::math::Vector3d force(wrench(0), wrench(1), wrench(2));
        ignition::math::Vector3d torque(wrench(3), wrench(4), wrench(5));

        frame->AddRelativeForce(force);
        frame->AddRelativeTorque(torque);
    }


    void HydrodynamicsPlugin::ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel, const Eigen::Matrix6d& _Ma, Eigen::Matrix6d &_Ca) const
    {
        // This corresponds to eq. 6.43 on p. 120 in
        // Fossen, Thor, "Handbook of Marine Craft and Hydrodynamics and Motion
        // Control", 2011
        Eigen::Vector6d ab = this->GetAddedMass() * _vel;
        Eigen::Matrix3d Sa = -1 * CrossProductOperator(ab.head<3>());
        _Ca << Eigen::Matrix3d::Zero(), Sa, 
               Sa, -1 * CrossProductOperator(ab.tail<3>());
    }


    void HydrodynamicsPlugin::ComputeDampingMatrix(const Eigen::Vector6d& _vel, Eigen::Matrix6d &_D) const
    {
        // From Antonelli 2014: the viscosity of the fluid causes
        // the presence of dissipative drag and lift forces on the
        // body. A common simplification is to consider only linear
        // and quadratic damping terms and group these terms in a
        // matrix Drb

        _D.setZero();

        _D = -1 * (this->linear_damping + this->offsetLinearDamping * Eigen::Matrix6d::Identity()) -
                    _vel[0] * (this->DLinForwardSpeed +
                    this->offsetLinForwardSpeedDamping * Eigen::Matrix6d::Identity());

        // Nonlinear damping matrix is considered as a diagonal matrix
        for (int i = 0; i < MAX_DIMENSION; i++)
        {
            _D(i, i) += -1 * (this->non_linear_damping(i, i) + this->offsetNonLinDamping) * std::fabs(_vel[i]);
        }
        _D *= this->scalingDamping;
    }


    Eigen::Matrix6d HydrodynamicsPlugin::GetAddedMass() const
    {
        Eigen::Matrix6d M_d = this->added_mass + this->offsetAddedMass * Eigen::Matrix6d::Identity();

        return this->scalingAddedMass * M_d;
    }

    // Constant vertical force on center of mass. 
    void HydrodynamicsPlugin::ApplyBuoyancyForce()
    {
        double vol = this->volume;
        double rho = this->fluid_density;
        ignition::math::Vector3d buoyancy_force;

        if (!this->is_neutrally_buoyant)
        {
            // the buoyant force: Fb = fluid_density * gravity * volume of fluid displaced
            buoyancy_force = -rho * vol * this->gravity * this->scalingBuoyancy;
        }
        else
        {
            buoyancy_force = -this->mass * this->gravity;
        }

        frame->AddForceAtRelativePosition(buoyancy_force, this->rel_CoB);

    }

} // namespace triton_gazebo
