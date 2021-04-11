#include "pos_vel_factor.hpp"

PosVelFactor::PosVelFactor(const double delta_t_) : delta_t(delta_t_), info_coeff(50) {}

bool PosVelFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Vector3d> pos_i(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> pos_j(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> vel_i(parameters[2]);
    Eigen::Map<const Eigen::Vector3d> vel_j(parameters[3]);
    Eigen::Map<Eigen::Vector3d> res(residuals);

    Eigen::Vector3d average_vel = 0.5 * (vel_i + vel_j);
    res = (pos_j - pos_i - average_vel * delta_t) * info_coeff;

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J_Pi(jacobians[0]);
            J_Pi.setZero();
            J_Pi.topLeftCorner<3, 3>() = -Eigen::Matrix3d::Identity() * info_coeff;
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J_Pj(jacobians[1]);
            J_Pj.setZero();
            J_Pj.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * info_coeff;
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J_Vi(jacobians[2]);
            J_Vi.setZero();
            J_Vi.topLeftCorner<3, 3>() = -0.5 * Eigen::Matrix3d::Identity() * delta_t * info_coeff;
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J_Vj(jacobians[3]);
            J_Vj.setZero();
            J_Vj.topLeftCorner<3, 3>() = -0.5 * Eigen::Matrix3d::Identity() * delta_t * info_coeff;
        }

    }
    
    return true;
}