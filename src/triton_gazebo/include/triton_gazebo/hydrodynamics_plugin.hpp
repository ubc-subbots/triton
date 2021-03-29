#ifndef TRITON_GAZEBO__HYDRODYNAMICS_PLUGIN
#define TRITON_GAZEBO__HYDRODYNAMICS_PLUGIN

#include <functional>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "include/gazebo_utils.hpp"
#include "include/math_utils.hpp"

namespace triton_gazebo 
{

    class HydrodynamicsPlugin : public gazebo::ModelPlugin
    {

    public:
        
        /// @brief Constructor
        HydrodynamicsPlugin ();

        /// @brief Destructor
        ~HydrodynamicsPlugin ();

    protected:

        /** 
         * @brief Load function called by Gazebo to initialize the plugin. 
         * 
         * 
         */
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

        /**
         * @brief We need to assemble the standard robotics equation of motion for the AUV
         * on each iteration. This can be done by gathering velocity and acceleration data
         * and using SDF parameters to approximate the hydrodynamic and hydrostatic forces
         */ 
        virtual void Update(const gazebo::common::UpdateInfo &_info);

    private:
        /**  
         * @brief Method to bind the Update function to the Gazebo Simulation update
         */
        void Connect(void);

        /**  
         * @brief Collect parameters specified in the world description (gravity, fluid density)
         * 
         * @param world_sdf A pointer to the world description
         */
        bool GetWorldParameters(sdf::ElementPtr world_sdf);

        /**  
         * @brief Collect parameters specified in the robot model description
         */
        bool GetLinkParameters(sdf::ElementPtr link_sdf);

        /**
         * @brief Get the current 6-dimensional velocity vector from gazebo. 
         * 
         * @return A vector containing linear and angular velocity
         */
        Eigen::Vector6d GetVelocityVector();

        /**
         * @brief Get the current 6-dimensional acceleration vector from gazebo. 
         * 
         * @return A vector containing linear and angular acceleration
         */
        Eigen::Vector6d GetAccelerationVector();

        /**
         * @brief Apply a 6-dimensional wrench to a Gazebo model by dividing components
         * into linear and angular ignition/math vectors and using Gazebo API to send command
         */
        void SetWrenchVector(Eigen::Vector6d wrench);

        /**
         * Computed matrices to satisfy dynamics equations
         */
        void ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel, const Eigen::Matrix6d& _Ma, Eigen::Matrix6d &_Ca) const;
        void ComputeDampingMatrix(const Eigen::Vector6d& _vel, Eigen::Matrix6d &_D) const;
        Eigen::Matrix6d GetAddedMass() const;
        double ComputeScalerDrag(double);
        void ApplyBuoyancyForce(void);


        gazebo::physics::ModelPtr model;
        gazebo::event::ConnectionPtr updateConnection_;
        // For now, apply to frame but we should Apply hydrodynamics to all links 
        gazebo::physics::LinkPtr frame;

        Eigen::Matrix6d added_mass;
        Eigen::Matrix6d coriolis_matrix;
        Eigen::Matrix6d damping_matrix;

        Eigen::Matrix6d DLinForwardSpeed;

        Eigen::Matrix6d linear_damping;
        Eigen::Matrix6d non_linear_damping;
        Eigen::Vector6d quadratic_damping;

        double mass;
        double fluid_density;
        double gravity;

        // Params pulled from Plankton, likely these will be negligable
        double scalingAddedMass; 
        double offsetAddedMass; 
        double scalingDamping;
        double offsetLinearDamping;
        double offsetLinForwardSpeedDamping;
        double offsetNonLinDamping;
        double volume;

        double length;
        double width;
        double height;

        double Cd;
    };

} //namespace triton_gazebo

GZ_REGISTER_MODEL_PLUGIN(triton_gazebo::HydrodynamicsPlugin)

#endif //TRITON_GAZEBO__HYDRODYNAMICS_PLUGIN