#ifndef GNSS_VI_INITIALIZER
#define GNSS_VI_INITIALIZER

#include <vector>
#include <eigen3/Eigen/Dense>

#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_spp.hpp>

#include "../feature_manager.h"

using namespace gnss_comm;

class GNSSVIInitializer
{
    public:
        GNSSVIInitializer(const std::vector<std::vector<ObsPtr>> &gnss_meas_buf_, 
            const std::vector<std::vector<EphemBasePtr>> &gnss_ephem_buf_, 
            const std::vector<double> &iono_params_);
        GNSSVIInitializer(const GNSSVIInitializer&) = delete;
        GNSSVIInitializer& operator=(const GNSSVIInitializer&) = delete;
        ~GNSSVIInitializer() {};

        // get a rough location by SPP
        bool coarse_localization(Eigen::Matrix<double, 7, 1> &result);
        bool yaw_alignment(const std::vector<Eigen::Vector3d> &local_vs, const Eigen::Vector3d &rough_anchor_ecef, 
            double &aligned_yaw, double &rcv_ddt);
        bool anchor_refinement(const std::vector<Eigen::Vector3d> &local_ps, 
            const double aligned_yaw, const double aligned_ddt, 
            const Eigen::Matrix<double, 7, 1> &rough_ecef_dt, Eigen::Matrix<double, 7, 1> &refined_ecef_dt);
    
    private:
        const std::vector<std::vector<ObsPtr>> &gnss_meas_buf;
        const std::vector<std::vector<EphemBasePtr>> &gnss_ephem_buf;
        const std::vector<double> &iono_params;

        uint32_t num_all_meas;
        std::vector<std::vector<SatStatePtr>> all_sat_states;

        static constexpr uint32_t MAX_ITERATION = 10;
        static constexpr double   CONVERGENCE_EPSILON = 1e-5;
};


#endif