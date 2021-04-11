#include "gnss_psr_dopp_factor.hpp"

GnssPsrDoppFactor::GnssPsrDoppFactor(const ObsPtr &_obs, const EphemBasePtr &_ephem, 
    std::vector<double> &_iono_paras, const double _ratio) 
        : obs(_obs), ephem(_ephem), iono_paras(_iono_paras), ratio(_ratio)
{
    freq = L1_freq(obs, &freq_idx);
    LOG_IF(FATAL, freq < 0) << "No L1 observation found.";

    uint32_t sys = satsys(obs->sat, NULL);
    double tof = obs->psr[freq_idx] / LIGHT_SPEED;
    gtime_t sv_tx = time_add(obs->time, -tof);

    if (sys == SYS_GLO)
    {
        GloEphemPtr glo_ephem = std::dynamic_pointer_cast<GloEphem>(ephem);
        svdt = geph2svdt(sv_tx, glo_ephem);
        sv_tx = time_add(sv_tx, -svdt);
        sv_pos = geph2pos(sv_tx, glo_ephem, &svdt);
        sv_vel = geph2vel(sv_tx, glo_ephem, &svddt);
        tgd = 0.0;
        pr_uura = 2.0 * (obs->psr_std[freq_idx]/0.16);
        dp_uura = 2.0 * (obs->dopp_std[freq_idx]/0.256);
    }
    else
    {
        EphemPtr eph = std::dynamic_pointer_cast<Ephem>(ephem);
        svdt = eph2svdt(sv_tx, eph);
        sv_tx = time_add(sv_tx, -svdt);
        sv_pos = eph2pos(sv_tx, eph, &svdt);
        sv_vel = eph2vel(sv_tx, eph, &svddt);
        tgd = eph->tgd[0];
        if (sys == SYS_GAL)
        {
            pr_uura = (eph->ura - 2.0) * (obs->psr_std[freq_idx]/0.16);
            dp_uura = (eph->ura - 2.0) * (obs->dopp_std[freq_idx]/0.256);
        }
        else
        {
            pr_uura = (eph->ura - 1.0) * (obs->psr_std[freq_idx]/0.16);
            dp_uura = (eph->ura - 1.0) * (obs->dopp_std[freq_idx]/0.256);
        }
    }

    LOG_IF(FATAL, pr_uura <= 0) << "pr_uura is " << pr_uura;
    LOG_IF(FATAL, dp_uura <= 0) << "dp_uura is " << dp_uura;
    relative_sqrt_info = 10.0;
}

bool GnssPsrDoppFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    double rcv_dt = parameters[4][0];
    double rcv_ddt = parameters[5][0];
    double yaw_diff = parameters[6][0];
    Eigen::Vector3d ref_ecef(parameters[7][0], parameters[7][1], parameters[7][2]);

    const Eigen::Vector3d local_pos = ratio*Pi + (1.0-ratio)*Pj;
    const Eigen::Vector3d local_vel = ratio*Vi + (1.0-ratio)*Vj;

    double sin_yaw_diff = std::sin(yaw_diff);
    double cos_yaw_diff = std::cos(yaw_diff);
    Eigen::Matrix3d R_enu_local;
    R_enu_local << cos_yaw_diff, -sin_yaw_diff, 0,
                   sin_yaw_diff,  cos_yaw_diff, 0,
                   0           ,  0           , 1;
    Eigen::Matrix3d R_ecef_enu = ecef2rotation(ref_ecef);
    Eigen::Matrix3d R_ecef_local = R_ecef_enu * R_enu_local;

    Eigen::Vector3d P_ecef = R_ecef_local * local_pos + ref_ecef;
    Eigen::Vector3d V_ecef = R_ecef_local * local_vel;

    double ion_delay = 0, tro_delay = 0;
    double azel[2] = {0, M_PI/2.0};
    if (P_ecef.norm() > 0)
    {
        sat_azel(P_ecef, sv_pos, azel);
        Eigen::Vector3d rcv_lla = ecef2geo(P_ecef);
        tro_delay = calculate_trop_delay(obs->time, rcv_lla, azel);
        ion_delay = calculate_ion_delay(obs->time, iono_paras, rcv_lla, azel);
    }
    double sin_el = sin(azel[1]);
    double sin_el_2 = sin_el*sin_el;
    double pr_weight = sin_el_2 / pr_uura * relative_sqrt_info;
    double dp_weight = sin_el_2 / dp_uura * relative_sqrt_info * PSR_TO_DOPP_RATIO;

    Eigen::Vector3d rcv2sat_ecef = sv_pos - P_ecef;
    Eigen::Vector3d rcv2sat_unit = rcv2sat_ecef.normalized();

    const double psr_sagnac = EARTH_OMG_GPS*(sv_pos(0)*P_ecef(1)-sv_pos(1)*P_ecef(0))/LIGHT_SPEED;
    double psr_estimated = rcv2sat_ecef.norm() + psr_sagnac + rcv_dt - svdt*LIGHT_SPEED + 
                                ion_delay + tro_delay + tgd*LIGHT_SPEED;
    
    residuals[0] = (psr_estimated - obs->psr[freq_idx]) * pr_weight;

    const double dopp_sagnac = EARTH_OMG_GPS/LIGHT_SPEED*(sv_vel(0)*P_ecef(1)+
            sv_pos(0)*V_ecef(1) - sv_vel(1)*P_ecef(0) - sv_pos(1)*V_ecef(0));
    double dopp_estimated = (sv_vel - V_ecef).dot(rcv2sat_unit) + dopp_sagnac + rcv_ddt - svddt*LIGHT_SPEED;
    const double wavelength = LIGHT_SPEED / freq;
    residuals[1] = (dopp_estimated + obs->dopp[freq_idx]*wavelength) * dp_weight;

    if (jacobians)
    {
        // J_Pi
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_Pi(jacobians[0]);
            J_Pi.setZero();
            J_Pi.topLeftCorner<1, 3>() = -rcv2sat_unit.transpose() * R_ecef_local * pr_weight * ratio;

            const double norm3 = pow(rcv2sat_ecef.norm(), 3);
            const double norm2 = rcv2sat_ecef.squaredNorm();
            Eigen::Matrix3d unit2rcv_pos;
            for (size_t i = 0; i < 3; ++i)
            {
                for (size_t j = 0; j < 3; ++j)
                {
                    if (i == j)
                        unit2rcv_pos(i, j) = (norm2-rcv2sat_ecef(i)*rcv2sat_ecef(i))/norm3;
                    else
                        unit2rcv_pos(i, j) = (-rcv2sat_ecef(i)*rcv2sat_ecef(j))/norm3;
                }
            }
            unit2rcv_pos *= -1;
            J_Pi.bottomLeftCorner<1, 3>() = (sv_vel-V_ecef).transpose() * unit2rcv_pos * 
                R_ecef_local * dp_weight * ratio;
        }

        // J_Vi
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J_Vi(jacobians[1]);
            J_Vi.setZero();
            J_Vi.bottomLeftCorner<1, 3>() = rcv2sat_unit.transpose() * (-1.0) * 
                R_ecef_local * dp_weight * ratio;
        }

        // J_Pj
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_Pj(jacobians[2]);
            J_Pj.setZero();
            J_Pj.topLeftCorner<1, 3>() = -rcv2sat_unit.transpose() * R_ecef_local * pr_weight * (1.0-ratio);

            const double norm3 = pow(rcv2sat_ecef.norm(), 3);
            const double norm2 = rcv2sat_ecef.squaredNorm();
            Eigen::Matrix3d unit2rcv_pos;
            for (size_t i = 0; i < 3; ++i)
            {
                for (size_t j = 0; j < 3; ++j)
                {
                    if (i == j)
                        unit2rcv_pos(i, j) = (norm2-rcv2sat_ecef(i)*rcv2sat_ecef(i))/norm3;
                    else
                        unit2rcv_pos(i, j) = (-rcv2sat_ecef(i)*rcv2sat_ecef(j))/norm3;
                }
            }
            unit2rcv_pos *= -1;
            J_Pj.bottomLeftCorner<1, 3>() = (sv_vel-V_ecef).transpose() * unit2rcv_pos * 
                R_ecef_local * dp_weight * (1.0-ratio);
        }

        // J_Vj
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J_Vj(jacobians[3]);
            J_Vj.setZero();
            J_Vj.bottomLeftCorner<1, 3>() = rcv2sat_unit.transpose() * (-1.0) * 
                R_ecef_local * dp_weight * (1.0-ratio);
        }

        // J_rcv_dt
        if (jacobians[4])
        {
            jacobians[4][0] = 1.0 * pr_weight;
            jacobians[4][1] = 0;
        }

        // J_rcv_ddt
        if (jacobians[5])
        {
            jacobians[5][0] = 0;
            jacobians[5][1] = 1.0 * dp_weight;
        }

        // J_yaw_diff
        if (jacobians[6])
        {
            Eigen::Matrix3d d_yaw;
            d_yaw << -sin_yaw_diff, -cos_yaw_diff, 0, 
                      cos_yaw_diff, -sin_yaw_diff, 0, 
                      0           ,  0           , 0;
            jacobians[6][0] = -rcv2sat_unit.dot(R_ecef_enu * d_yaw * local_pos) * pr_weight;
            jacobians[6][1] = -rcv2sat_unit.dot(R_ecef_enu * d_yaw * local_vel) * dp_weight;
        }

        // J_ref_ecef, approximation for simplicity
        if (jacobians[7])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_ref_ecef(jacobians[7]);
            J_ref_ecef.setZero();
            J_ref_ecef.row(0) = -rcv2sat_unit.transpose() * pr_weight;
        }
    }
    return true;
}