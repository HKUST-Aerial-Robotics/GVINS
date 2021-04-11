#include "gnss_dt_anchor_factor.hpp"

bool DtAnchorFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    double rev_dt_i  = parameters[0][0];

    residuals[0] = (rev_dt_i - 0.0) * dt_anchor_coeff_;     // anchor to 0

    if (jacobians)
    {
        if (jacobians[0])
        {
            jacobians[0][0] = dt_anchor_coeff_;
        }
    }

    return true;
}