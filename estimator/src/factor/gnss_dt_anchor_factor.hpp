#ifndef DT_ANCHOR_FACTOR_H_
#define DT_ANCHOR_FACTOR_H_

#include <ceres/ceres.h>

/* 
**  parameters[0]: rev_dt (t)   in light travelling distance (m)
 */
class DtAnchorFactor : public ceres::SizedCostFunction<1, 1>
{
    public: 
        DtAnchorFactor(const double dt_anchor_coeff=1000) : dt_anchor_coeff_(dt_anchor_coeff) {}
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    private:
        double dt_anchor_coeff_;
};

#endif