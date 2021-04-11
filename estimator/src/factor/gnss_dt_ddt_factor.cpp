#include "gnss_dt_ddt_factor.hpp"

DtDdtFactor::DtDdtFactor(const double delta_t_) : delta_t(delta_t_), dt_info_coeff(50)  {} 

bool DtDdtFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    double rev_dt_i  = parameters[0][0];
    double rev_dt_j  = parameters[1][0];
    double rev_ddt_i = parameters[2][0];
    double rev_ddt_j = parameters[3][0];

    double average_ddt = 0.5 * (rev_ddt_i + rev_ddt_j);
    residuals[0] = (rev_dt_j - rev_dt_i - average_ddt * delta_t) * dt_info_coeff;

    if (jacobians)
    {
        if (jacobians[0])
        {
            jacobians[0][0] = -dt_info_coeff;
        }
        if (jacobians[1])
        {
            jacobians[1][0] = dt_info_coeff;
        }
        if (jacobians[2])
        {
            jacobians[2][0] = -0.5 * delta_t * dt_info_coeff;
        }
        if (jacobians[3])
        {
            jacobians[3][0] = -0.5 * delta_t * dt_info_coeff;
        }
    }

    return true;
}