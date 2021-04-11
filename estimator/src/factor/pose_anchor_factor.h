#ifndef POSE_ANCHOR_FACTOR_H_
#define POSE_ANCHOR_FACTOR_H_

#include <Eigen/Dense>
#include <ceres/ceres.h>

/* 
**  parameters[0]: pose which needs to be anchored to a constant value
 */
class PoseAnchorFactor : public ceres::SizedCostFunction<6, 7>
{
    public: 
        PoseAnchorFactor() = delete;
        PoseAnchorFactor(const std::vector<double> anchor_value);
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    private:
        Eigen::Matrix<double, 7, 1> _anchor_point;
        constexpr static double sqrt_info = 120;
};

#endif