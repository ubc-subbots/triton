#ifndef TRITON_GAZEBO__HYDRODYNAMICS_PLUGIN
#define TRITON_GAZEBO__HYDRODYNAMICS_PLUGIN

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace triton_gazebo 
{
    class HydrodynamicsPlugin : public gazebo::ModelPlugin
    {

    public:

        /** Constructor
         */
        HydrodynamicsPlugin ();

        /** Destructor
         */
        ~HydrodynamicsPlugin ();

    protected:

        /** Plugin load method
         * 
         * 
         */
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

        /** Plugin update method
         * 
         * 
         */ 
        virtual void OnUpdate();

    private:
        gazebo::physics::ModelPtr model_;
        gazebo::event::ConnectionPtr updateConnection_;
    
    };
} //namespace triton_gazebo

GZ_REGISTER_MODEL_PLUGIN(triton_gazebo::HydrodynamicsPlugin)

#endif //TRITON_GAZEBO__HYDRODYNAMICS_PLUGIN