#include "gnss_ddt_smooth_factor.hpp"

bool DdtSmoothFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    double rcv_ddt_i  = parameters[0][0];
    double rcv_ddt_j  = parameters[1][0];

    residuals[0] = (rcv_ddt_i - rcv_ddt_j) * weight_;

    if (jacobians)
    {
        if (jacobians[0])
        {
            jacobians[0][0] = weight_;
        }
        if (jacobians[1])
        {
            jacobians[1][0] = -weight_;
        }
    }
    return true;
}