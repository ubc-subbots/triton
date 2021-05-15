#include "include/math_utils.hpp"

namespace triton_gazebo
{

    Eigen::Matrix6d ToDiagonalMatrix(Eigen::Vector6d _vec)
    {
        Eigen::Matrix6d diag_mat = Eigen::Matrix6d::Identity();
           
        for (int i = 0; i < MAX_DIMENSION; i++)
        {
            diag_mat(i, i) = _vec(i);
        }

        return diag_mat;
    }


    Eigen::Matrix3d CrossProductOperator(Eigen::Vector3d _x)
    {
        Eigen::Matrix3d output;
        output << 0.0, -_x(2), _x(1), _x(2), 0.0, -_x(0), -_x(1), _x(0), 0.0;
        return output;
    }


    Eigen::Vector6d ToNED(Eigen::Vector6d _vec)
    {
        Eigen::Vector6d ned_vec;
        ned_vec << _vec(0), -_vec(1), -_vec(2), _vec(3), -_vec(4), -_vec(5);
        return ned_vec;
    }


    Eigen::Vector6d FromNED(Eigen::Vector6d _vec)
    {
        return ToNED(_vec);
    }

}