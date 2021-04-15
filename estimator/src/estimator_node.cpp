#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gvins/LocalSensorExternalTrigger.h>
#include <sensor_msgs/NavSatFix.h>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

using namespace gnss_comm;

#define MAX_GNSS_CAMERA_DELAY 0.05

std::unique_ptr<Estimator> estimator_ptr;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<std::vector<ObsPtr>> gnss_meas_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = -1;

std::mutex m_time;
double next_pulse_time;
bool next_pulse_time_valid;
double time_diff_gnss_local;
bool time_diff_valid;
double latest_gnss_time;
double last_feature_time;
uint64_t feature_counter;
int skip_parameter;

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator_ptr->g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator_ptr->g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator_ptr->Ps[WINDOW_SIZE];
    tmp_Q = estimator_ptr->Rs[WINDOW_SIZE];
    tmp_V = estimator_ptr->Vs[WINDOW_SIZE];
    tmp_Ba = estimator_ptr->Bas[WINDOW_SIZE];
    tmp_Bg = estimator_ptr->Bgs[WINDOW_SIZE];
    acc_0 = estimator_ptr->acc_0;
    gyr_0 = estimator_ptr->gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

bool
getMeasurements(std::vector<sensor_msgs::ImuConstPtr> &imu_msg, sensor_msgs::PointCloudConstPtr &img_msg, std::vector<ObsPtr> &gnss_msg)
{
    if (imu_buf.empty() || feature_buf.empty() || (GNSS_ENABLE && gnss_meas_buf.empty()))
        return false;
    
    double front_feature_ts = feature_buf.front()->header.stamp.toSec();

    if (!(imu_buf.back()->header.stamp.toSec() > front_feature_ts))
    {
        //ROS_WARN("wait for imu, only should happen at the beginning");
        sum_of_wait++;
        return false;
    }
    double front_imu_ts = imu_buf.front()->header.stamp.toSec();
    while (!feature_buf.empty() && front_imu_ts > front_feature_ts)
    {
        ROS_WARN("throw img, only should happen at the beginning");
        feature_buf.pop();
        front_feature_ts = feature_buf.front()->header.stamp.toSec();
    }

    if (GNSS_ENABLE)
    {
        front_feature_ts += time_diff_gnss_local;
        double front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
        while (!gnss_meas_buf.empty() && front_gnss_ts < front_feature_ts-MAX_GNSS_CAMERA_DELAY)
        {
            ROS_WARN("throw gnss, only should happen at the beginning");
            gnss_meas_buf.pop();
            if (gnss_meas_buf.empty()) return false;
            front_gnss_ts = time2sec(gnss_meas_buf.front()[0]->time);
        }
        if (gnss_meas_buf.empty())
        {
            ROS_WARN("wait for gnss...");
            return false;
        }
        else if (abs(front_gnss_ts-front_feature_ts) < MAX_GNSS_CAMERA_DELAY)
        {
            gnss_msg = gnss_meas_buf.front();
            gnss_meas_buf.pop();
        }
    }

    img_msg = feature_buf.front();
    feature_buf.pop();

    while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator_ptr->td)
    {
        imu_msg.emplace_back(imu_buf.front());
        imu_buf.pop();
    }
    imu_msg.emplace_back(imu_buf.front());
    if (imu_msg.empty())
        ROS_WARN("no imu between two image");
    return true;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator_ptr->solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

void gnss_ephem_callback(const GnssEphemMsgConstPtr &ephem_msg)
{
    EphemPtr ephem = msg2ephem(ephem_msg);
    estimator_ptr->inputEphem(ephem);
}

void gnss_glo_ephem_callback(const GnssGloEphemMsgConstPtr &glo_ephem_msg)
{
    GloEphemPtr glo_ephem = msg2glo_ephem(glo_ephem_msg);
    estimator_ptr->inputEphem(glo_ephem);
}

void gnss_iono_params_callback(const StampedFloat64ArrayConstPtr &iono_msg)
{
    double ts = iono_msg->header.stamp.toSec();
    std::vector<double> iono_params;
    std::copy(iono_msg->data.begin(), iono_msg->data.end(), std::back_inserter(iono_params));
    assert(iono_params.size() == 8);
    estimator_ptr->inputIonoParams(ts, iono_params);
}

void gnss_meas_callback(const GnssMeasMsgConstPtr &meas_msg)
{
    std::vector<ObsPtr> gnss_meas = msg2meas(meas_msg);

    latest_gnss_time = time2sec(gnss_meas[0]->time);

    // cerr << "gnss ts is " << std::setprecision(20) << time2sec(gnss_meas[0]->time) << endl;
    if (!time_diff_valid)   return;

    m_buf.lock();
    gnss_meas_buf.push(std::move(gnss_meas));
    m_buf.unlock();
    con.notify_one();
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    ++ feature_counter;

    if (time_diff_valid)
    {
        const double this_feature_ts = feature_msg->header.stamp.toSec()+time_diff_gnss_local;
        if (skip_parameter < 0 && latest_gnss_time > 0 && last_feature_time > 0)
        {
            if (abs(this_feature_ts - latest_gnss_time) > abs(last_feature_time - latest_gnss_time))
                skip_parameter = feature_counter%2;       // skip this frame and afterwards
            else
                skip_parameter = 1 - (feature_counter%2);   // skip next frame and afterwards
        }
        // cerr << "feature counter is " << feature_counter << ", skip parameter is " << int(skip_parameter) << endl;
        last_feature_time = this_feature_ts;
    }

    if (!GNSS_ENABLE || (skip_parameter >= 0 && int(feature_counter%2) != skip_parameter))
    {
        m_buf.lock();
        feature_buf.push(feature_msg);
        m_buf.unlock();
        con.notify_one();
    }
}

void local_trigger_info_callback(const gvins::LocalSensorExternalTriggerConstPtr &trigger_msg)
{
    std::lock_guard<std::mutex> lg(m_time);

    if (next_pulse_time_valid)
    {
        time_diff_gnss_local = next_pulse_time - trigger_msg->header.stamp.toSec();
        estimator_ptr->inputGNSSTimeDiff(time_diff_gnss_local);
        if (!time_diff_valid)       // just get calibrated
            std::cout << "time difference between GNSS and VI-Sensor got calibrated\n";
        // cerr << "time diff is " << std::setprecision(15) << time_diff_gnss_local << std::endl;
        time_diff_valid = true;
    }
}

void gnss_tp_info_callback(const GnssTimePulseInfoMsgConstPtr &tp_msg)
{
    gtime_t tp_time = gpst2time(tp_msg->time.week, tp_msg->time.tow);
    if (tp_msg->utc_based || tp_msg->time_sys == SYS_GLO)
        tp_time = utc2gpst(tp_time);
    else if (tp_msg->time_sys == SYS_GAL)
        tp_time = gst2time(tp_msg->time.week, tp_msg->time.tow);
    else if (tp_msg->time_sys == SYS_BDS)
        tp_time = bdt2time(tp_msg->time.week, tp_msg->time.tow);
    else if (tp_msg->time_sys == SYS_NONE)
    {
        std::cerr << "Unknown time system in GNSSTimePulseInfoMsg.\n";
        return;
    }
    double gnss_ts = time2sec(tp_time);

    std::lock_guard<std::mutex> lg(m_time);
    next_pulse_time = gnss_ts;
    next_pulse_time_valid = true;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator_ptr->clearState();
        estimator_ptr->setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::vector<sensor_msgs::ImuConstPtr> imu_msg;
        sensor_msgs::PointCloudConstPtr img_msg;
        std::vector<ObsPtr> gnss_msg;

        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
                    return getMeasurements(imu_msg, img_msg, gnss_msg);
                 });
        lk.unlock();
        m_estimator.lock();
        double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
        for (auto &imu_data : imu_msg)
        {
            double t = imu_data->header.stamp.toSec();
            double img_t = img_msg->header.stamp.toSec() + estimator_ptr->td;
            if (t <= img_t)
            { 
                if (current_time < 0)
                    current_time = t;
                double dt = t - current_time;
                ROS_ASSERT(dt >= 0);
                current_time = t;
                dx = imu_data->linear_acceleration.x;
                dy = imu_data->linear_acceleration.y;
                dz = imu_data->linear_acceleration.z;
                rx = imu_data->angular_velocity.x;
                ry = imu_data->angular_velocity.y;
                rz = imu_data->angular_velocity.z;
                estimator_ptr->processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

            }
            else
            {
                double dt_1 = img_t - current_time;
                double dt_2 = t - img_t;
                current_time = img_t;
                ROS_ASSERT(dt_1 >= 0);
                ROS_ASSERT(dt_2 >= 0);
                ROS_ASSERT(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                dx = w1 * dx + w2 * imu_data->linear_acceleration.x;
                dy = w1 * dy + w2 * imu_data->linear_acceleration.y;
                dz = w1 * dz + w2 * imu_data->linear_acceleration.z;
                rx = w1 * rx + w2 * imu_data->angular_velocity.x;
                ry = w1 * ry + w2 * imu_data->angular_velocity.y;
                rz = w1 * rz + w2 * imu_data->angular_velocity.z;
                estimator_ptr->processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
            }
        }

        if (GNSS_ENABLE && !gnss_msg.empty())
            estimator_ptr->processGNSS(gnss_msg);

        ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

        TicToc t_s;
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
        for (unsigned int i = 0; i < img_msg->points.size(); i++)
        {
            int v = img_msg->channels[0].values[i] + 0.5;
            int feature_id = v / NUM_OF_CAM;
            int camera_id = v % NUM_OF_CAM;
            double x = img_msg->points[i].x;
            double y = img_msg->points[i].y;
            double z = img_msg->points[i].z;
            double p_u = img_msg->channels[1].values[i];
            double p_v = img_msg->channels[2].values[i];
            double velocity_x = img_msg->channels[3].values[i];
            double velocity_y = img_msg->channels[4].values[i];
            ROS_ASSERT(z == 1);
            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
        estimator_ptr->processImage(image, img_msg->header);

        double whole_t = t_s.toc();
        printStatistics(*estimator_ptr, whole_t);
        std_msgs::Header header = img_msg->header;
        header.frame_id = "world";

        pubOdometry(*estimator_ptr, header);
        pubKeyPoses(*estimator_ptr, header);
        pubCameraPose(*estimator_ptr, header);
        pubPointCloud(*estimator_ptr, header);
        pubTF(*estimator_ptr, header);
        pubKeyframe(*estimator_ptr);
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator_ptr->solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gvins");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator_ptr.reset(new Estimator());
    estimator_ptr->setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    registerPub(n);

    next_pulse_time_valid = false;
    time_diff_valid = false;
    latest_gnss_time = -1;
    last_feature_time = -1;
    feature_counter = 0;
    skip_parameter = -1;

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_feature = n.subscribe("/gvins_feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/gvins_feature_tracker/restart", 2000, restart_callback);

    ros::Subscriber sub_ephem, sub_glo_ephem, sub_gnss_meas, sub_gnss_iono_params;
    ros::Subscriber sub_gnss_time_pluse_info, sub_local_trigger_info;
    if (GNSS_ENABLE)
    {
        sub_ephem = n.subscribe(GNSS_EPHEM_TOPIC, 100, gnss_ephem_callback);
        sub_glo_ephem = n.subscribe(GNSS_GLO_EPHEM_TOPIC, 100, gnss_glo_ephem_callback);
        sub_gnss_meas = n.subscribe(GNSS_MEAS_TOPIC, 100, gnss_meas_callback);
        sub_gnss_iono_params = n.subscribe(GNSS_IONO_PARAMS_TOPIC, 100, gnss_iono_params_callback);

        if (GNSS_LOCAL_ONLINE_SYNC)
        {
            sub_gnss_time_pluse_info = n.subscribe(GNSS_TP_INFO_TOPIC, 100, 
                gnss_tp_info_callback);
            sub_local_trigger_info = n.subscribe(LOCAL_TRIGGER_INFO_TOPIC, 100, 
                local_trigger_info_callback);
        }
        else
        {
            time_diff_gnss_local = GNSS_LOCAL_TIME_DIFF;
            estimator_ptr->inputGNSSTimeDiff(time_diff_gnss_local);
            time_diff_valid = true;
        }
    }

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
