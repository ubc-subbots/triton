#ifndef TRITON_GAZEBO__MATH_UTILS
#define TRITON_GAZEBO__MATH_UTILS

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#define MAX_DIMENSION 6

/**
 * @brief Shorthand definitions for 6 dimensional matricies and vectors (double precision)
 * 
 */
namespace Eigen
{
    typedef Matrix<double, 6, 6> Matrix6d;
    typedef Matrix<double, 6, 1> Vector6d;
} // namespace Eigen


namespace triton_gazebo
{

    /**
     * @brief Converts a vector to a 6-dimensional diagonal matrix
     * 
     */
    Eigen::Matrix6d ToDiagonalMatrix(Eigen::Vector6d _vec);

    /** 
     * @brief Function to compute the skew operator (cross product operator)
     * 
     * ex: given vectors y and z, this operation creates a matrix A s.t.
     *     y x z = A * z i.e. (y x) = A
     * 
     * @param _x    A 3-dimensional vector
     * 
     * @return      A 3-dimensional matrx representing the skew of _x
     *
     */
    Eigen::Matrix3d CrossProductOperator(Eigen::Vector3d _x);


    /** 
     * @brief Conversion to NED coordinates
     * 
     * @param _vec  A 6-dimensional vector
     * 
     * @return      _vec represented in NED coordinate frame
     *
     */
    Eigen::Vector6d ToNED(Eigen::Vector6d _vec);


    Eigen::Vector6d FromNED(Eigen::Vector6d _vec);

}

#endif // TRITON_GAZEBO__MATH_UTILS
