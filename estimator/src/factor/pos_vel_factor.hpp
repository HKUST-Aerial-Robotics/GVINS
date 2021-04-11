#ifndef POS_VEL_FACTOR_H_
#define POS_VEL_FACTOR_H_

#include <map>
#include <Eigen/Dense>
#include <ceres/ceres.h>

/* 
**  parameters[0]: position(k) in ECEF
**  parameters[1]: position(k+1) in ECEF
**  parameters[2]: velocity(k) in ECEF
**  parameters[3]: velocity(k+1) in ECEF
 */
class PosVelFactor : public ceres::SizedCostFunction<3, 7, 7, 9, 9>
{
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PosVelFactor() = delete;
        PosVelFactor(const double delta_t_);
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    private:
        double delta_t;
        double info_coeff;
};

#endif