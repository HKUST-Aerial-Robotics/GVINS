#include "pose_anchor_factor.h"

PoseAnchorFactor::PoseAnchorFactor(const std::vector<double> anchor_value)
{
    for (int i = 0; i < 7; ++i)     _anchor_point(i) = anchor_value[i];
}

bool PoseAnchorFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> pose(parameters[0]);
    Eigen::Map<Eigen::Matrix<double, 6, 1>> res(residuals);
    res.head<3>() = pose.head<3>() - _anchor_point.head<3>();
    const Eigen::Quaterniond curr_q(pose.tail<4>());
    const Eigen::Quaterniond anchor_q(_anchor_point.tail<4>());
    res.tail<3>() = 2.0 * (curr_q*anchor_q.inverse()).vec();
    res *= sqrt_info;
    if (jacobians && jacobians[0])
    {
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobians[0]);
        J.setZero();
        J.topLeftCorner<3, 3>().setIdentity();

        Eigen::Quaterniond anchor_q_inv = anchor_q.inverse();
        Eigen::Matrix3d J_q;
        J_q << anchor_q_inv.w(),  anchor_q_inv.z(), -anchor_q_inv.y(),
              -anchor_q_inv.z(),  anchor_q_inv.w(),  anchor_q_inv.x(),
               anchor_q_inv.y(), -anchor_q_inv.x(),  anchor_q_inv.w();
        J.block<3, 3>(3, 3) = J_q;
        J *= 2.0*sqrt_info;
    }
    return true;
}