#ifndef TRITON_GAZEBO__GAZEBO_UTILS
#define TRITON_GAZEBO__GAZEBO_UTILS

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "include/math_utils.hpp"


namespace triton_gazebo
{

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
    Eigen::Vector6d GetSdfVector(bool* status, sdf::ElementPtr _sdf, std::string param);


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
    Eigen::Matrix6d GetSdfMatrix(bool* status, sdf::ElementPtr _sdf, std::string param);

}

 #endif // TRITON_GAZEBO__GAZEBO_UTILS