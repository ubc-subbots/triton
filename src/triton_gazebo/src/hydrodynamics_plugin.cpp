#include "triton_gazebo/hydrodynamics_plugin.hpp"

namespace triton_gazebo
{

    /// @todo Create gazebo utility API to store these functions. 

    /**
     * @brief Collects a parameter from an sdf description, 
     * assigns a default value if not specified. 
     * 
     * @param status    Status indicating if the param was specified
     * @param _sdf      A pointer to the model's SDF description
     * @param param     The name of the parameter being located
     * 
     */
    template <typename parameter>
    parameter GetSdfElement(bool* status, sdf::ElementPtr _sdf, std::string param, const parameter def=(parameter)NULL)
    {
        parameter value = def;
        *status = false;

        if (_sdf->HasElement(param))
        {
            value = _sdf->Get<parameter>(param);
            *status = true;
        }

        return value;
    }


    /**
     * @brief Creates a 6x1 vector from a list of values in an SDF file. 
     * 
     * @todo defaul condition.
     * 
     * @param status    Status indicating if the param was specified
     * @param _sdf      A pointer to the model's SDF description
     * @param param     The name of the parameter being located
     * 
     */
    Eigen::Vector6d GetSdfVector(bool* status, sdf::ElementPtr _sdf, std::string param)
    {
        Eigen::Vector6d _vector;
        int idx = 0;
        double val;

        std::string sdf_data = GetSdfElement<std::string>(status, _sdf, param, "");

        if (*status == false) { return _vector; }

        std::istringstream iss(sdf_data);

        while (iss >> val)
        {
            _vector(idx++, 1) = val;
        }
        if (idx != MAX_DIMENSION) 
        {
            gzerr << "Vector did not fully populate, continuing...\n";
        }
        *status = true;

        return _vector;
    }


    /**
     * @brief Creates a 6x6 matrix from a list of values in an SDF file. 
     * 
     * @todo defaul condition.
     * 
     * @param status    Status indicating if the param was specified
     * @param _sdf      A pointer to the model's SDF description
     * @param param     The name of the parameter being located
     * 
     */
    Eigen::Matrix6d GetSdfMatrix(bool* status, sdf::ElementPtr _sdf, std::string param)
    {
        Eigen::Matrix6d _matrix;
        int r_idx = 0, c_idx = 0;
        double val;

        std::string sdf_data = GetSdfElement<std::string>(status, _sdf, param, "");

        if (*status == false) { return _matrix; }

        std::istringstream iss(sdf_data);

        while (iss >> val)
        {
            _matrix(r_idx, c_idx++) = val;
            if (c_idx == MAX_DIMENSION)
            {
                c_idx = 0;
                r_idx++;
            }
        }
        if (r_idx != MAX_DIMENSION) 
        {
            gzerr << "Matrix did not fully populate, continuing...\n";
        }
        *status = true;

        return _matrix;
    }


    /// @todo Create math header for these. 

    inline Eigen::Matrix3d CrossProductOperator(Eigen::Vector3d _x)
    {
        Eigen::Matrix3d output;
        output << 0.0, -_x(2), _x(1), _x(2), 0.0, -_x(0), -_x(1), _x(0), 0.0;
        return output;
    }


    Eigen::Vector6d HydrodynamicsPlugin::ToNED(Eigen::Vector6d _vec)
    {
        Eigen::Vector6d ned_vec;
        ned_vec << _vec(0), -_vec(1), -_vec(2), _vec(3), -_vec(4), -_vec(5);
        return ned_vec;
    }


    Eigen::Vector6d HydrodynamicsPlugin::FromNED(Eigen::Vector6d _vec)
    {
        return this->ToNED(_vec);
    }


    HydrodynamicsPlugin::HydrodynamicsPlugin() {}


    HydrodynamicsPlugin::~HydrodynamicsPlugin() {}


    void HydrodynamicsPlugin::Load(gazebo::physics::ModelPtr _model, 
                                   sdf::ElementPtr _sdf)
    {
        model = _model;
        frame = model->GetLink("cube::frame");
        this->mass = frame->GetInertial()->Mass();

        GetWorldParameters(_model->GetWorld()->SDF());

        GetLinkParameters(frame->GetSDF());

        this->DLinForwardSpeed.setZero();

        this->Connect();
    }

    // Left as bool in case more parameters are added later
    bool HydrodynamicsPlugin::GetWorldParameters(sdf::ElementPtr world_sdf)
    {
        bool status = true; // Not checked as we have default values in place
    
        this->fluid_density = GetSdfElement<double>(&status, world_sdf, "fluid_density", 1028.00);

        this->gravity = GetSdfElement<double>(&status, world_sdf, "gravity", 9.81);

        return status;
    }


    bool HydrodynamicsPlugin::GetLinkParameters(sdf::ElementPtr link_sdf)
    {
        sdf::ElementPtr hydro_model = link_sdf->GetElement("hydrodynamic_model");
        bool status = true;

        this->scalingAddedMass = GetSdfElement<double>(&status, hydro_model, "scalingAddedMass", 1.00);
        this->offsetAddedMass = GetSdfElement<double>(&status, hydro_model, "offsetAddedMass", 0.00);

        this->scalingDamping = GetSdfElement<double>(&status, hydro_model, "scalingDamping", 1.00);
        this->offsetLinearDamping = GetSdfElement<double>(&status, hydro_model, "offetLineaDamping", 0.00);
        this->offsetLinForwardSpeedDamping = GetSdfElement<double>(&status, hydro_model, "offsetLinForwardSpeedDamping", 0.00);

        this->offsetNonLinDamping = GetSdfElement<double>(&status, hydro_model, "offsetNonLinDamping", 0.00);

        this->added_mass = GetSdfMatrix(&status, hydro_model, "added_mass");

        if (!status)
        {
            gzerr << "Added mass matrix not specified.\n";
            exit(1);
        }

        Eigen::Vector6d linear_damping_vec = GetSdfVector(&status, hydro_model, "linear_damping");
        this->linear_damping = Eigen::Matrix6d::Identity();

        if (!status)
        {
            gzerr << "Linear damping not specified.\n";
            exit(1);
        }

        for (int i = 0; i < MAX_DIMENSION; i++)
        {
            this->linear_damping(i, i) = linear_damping_vec(i);
        }

        this->quadratic_damping = GetSdfVector(&status, hydro_model, "quadratic_damping");
        this->non_linear_damping = Eigen::Matrix6d::Identity();

        if (!status)
        {
            gzerr << "Quadratic damping not specified.\n";
            exit(1);
        }

        for (int i = 0; i < MAX_DIMENSION; i++)
        {
            this->non_linear_damping(i, i) = quadratic_damping(i);
        }

        return status;
    }


    void HydrodynamicsPlugin::Connect()
    {
        updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&HydrodynamicsPlugin::Update, this, std::placeholders::_1));
    }


    void HydrodynamicsPlugin::Update(const gazebo::common::UpdateInfo &_info)
    {
        Eigen::Vector6d velocity = this->GetVelocityVector();
        Eigen::Vector6d acceleration = this->GetAccelerationVector(); // should we compute this??

        //Eigen::Vector6d velRel = this->ToNED(velocity);
        Eigen::Vector6d velRel = velocity;

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
        Eigen::Vector6d added = -Ma * acceleration;//this->filteredAcc;

        // Added Coriolis term
        Eigen::Vector6d cor = -this->coriolis_matrix * velRel;

        // All additional (compared to standard rigid body) Fossen terms combined.
        Eigen::Vector6d tau = damping + added + cor;

        GZ_ASSERT(!std::isnan(tau.norm()), "Hydrodynamic forces vector is nan");

        if (!std::isnan(tau.norm()))
        {
            // this->SetWrenchVector(this->FromNED(tau));
            this->SetWrenchVector(tau);
        }
        //this->ApplyBuoyancyForce();
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
            /// @todo _vel is causing this to grow exponentially 
            _D(i, i) += -1 * (this->non_linear_damping(i, i) + this->offsetNonLinDamping) * std::fabs(_vel[i]);
        }
        _D *= this->scalingDamping;
    }


    Eigen::Matrix6d HydrodynamicsPlugin::GetAddedMass() const
    {
        Eigen::Matrix6d M_d = this->added_mass + this->offsetAddedMass * Eigen::Matrix6d::Identity();

        return this->scalingAddedMass * M_d;
    }

    // Specific to cube object
    double HydrodynamicsPlugin::ComputeScalerDrag(double cross_section_area)
    {
        return -0.5 * this->Cd * cross_section_area * this->fluid_density;
    }

} // namespace triton_gazebo
