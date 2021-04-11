
#ifndef DT_DDT_FACTOR_H_
#define DT_DDT_FACTOR_H_

#include <map>
#include <Eigen/Dense>
#include <ceres/ceres.h>

/* 
**  parameters[0]: rev_dt (t)   in light travelling distance (m)
**  parameters[1]: rev_dt (t+1) in light travelling distance (m)
**  parameters[2]: rev_ddt(t)   in light travelling distance per second (m/s)
**  parameters[3]: rev_ddt(t+1) in light travelling distance per second (m/s)
 */
class DtDdtFactor : public ceres::SizedCostFunction<1, 1, 1, 1, 1>
{
    public: 
        DtDdtFactor() = delete;
        DtDdtFactor(const double delta_t_);
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    private:
        double delta_t;
        double dt_info_coeff;
};

#endif