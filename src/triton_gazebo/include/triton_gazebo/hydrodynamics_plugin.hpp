#ifndef TRITON_GAZEBO__HYDRODYNAMICS_PLUGIN
#define TRITON_GAZEBO__HYDRODYNAMICS_PLUGIN

#include <functional>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

const double fluid_density = 1028.00;
const double gravity = 9.81;

namespace triton_gazebo 
{
    class HydrodynamicsPlugin : public gazebo::ModelPlugin
    {

    public:
        HydrodynamicsPlugin ();
        ~HydrodynamicsPlugin ();

    protected:
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
        virtual void Update(const gazebo::common::UpdateInfo &_info);

    private:
        /**  
         * Method to bind the Update function to the Gazebo Simulation update
         */
        void Connect();
        void GetTimeStep();
        void GetVelocityVector();
        void GetAccelerationVector();

        /**
         * Functions borrowed from Plankton.io
         */
        ComputeAddedCoriolisMatrix(const Eigen::Vector6d&,
                                   const Eigen::Matrix6d&,
                                   Eigen::Matrix6d);
        Eigen::Matrix6d GetAddedMass();
        void ComputeDampingMatrix(const Eigen::Vector6d,
                                  Eigen::Matrix6d);

        gazebo::physics::ModelPtr model;
        gazebo::event::ConnectionPtr updateConnection_;
        // Should Apply hydrodynamics to all links 
        gazebo::physics::LinkPtr frame;
        //std::vector<gazebo::physics::LinkPtr> links;

        Eigen::VectorXd velocity;
        Eigen::VectorXd acceleration;
    };
} //namespace triton_gazebo

GZ_REGISTER_MODEL_PLUGIN(triton_gazebo::HydrodynamicsPlugin)

#endif //TRITON_GAZEBO__HYDRODYNAMICS_PLUGIN