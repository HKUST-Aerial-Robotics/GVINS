# GVINS

GVINS: Tightly Coupled GNSS-Visual-Inertial Fusion for Smooth and Consistent State Estimation. [paper link](https://arxiv.org/pdf/2103.07899.pdf)

![](./figures/system_snapshot.png)

**GVINS** is a non-linear optimization based system that tightly fuses GNSS raw measurements with visual and inertial information for real-time and drift-free state estimation. By incorporating GNSS pseudorange and Doppler shift measurements, GVINS is capable to provide smooth and consistent 6-DoF global localization in complex environment. The system framework and VIO part are adapted from [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono). Our system contains the following features:

- global 6-DoF estimation in ECEF frame;
- multi-constellation support (GPS, GLONASS, Galileo, BeiDou);
- online local-ENU frame alignment;
- global pose recovery in GNSS-unfriendly or even GNSS-denied area.

## 1. Prerequisites
### 1.1 C++11 Compiler
This package requires some features of C++11.

### 1.2 ROS
This package is developed under [ROS Kinetic](http://wiki.ros.org/kinetic) environment.

### 1.3 Eigen
Our code uses [Eigen 3.3.3](https://gitlab.com/libeigen/eigen/-/archive/3.3.3/eigen-3.3.3.zip) for matrix manipulation.

### 1.4 Ceres
We use [ceres](https://ceres-solver.googlesource.com/ceres-solver) 1.12.0 to solve the non-linear optimization problem.

### 1.5 gnss_comm
This package also requires [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) for ROS message definitions and some utility functions. Follow [those instructions](https://github.com/HKUST-Aerial-Robotics/gnss_comm) to build the *gnss_comm* package.

## 2. Build GVINS
Clone the repository to your catkin workspace (for example `~/catkin_ws/`):
```
cd ~/catkin_ws/src/
git clone https://github.com/HKUST-Aerial-Robotics/GVINS.git
```
Then build the package with:
```
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 3. Run GVINS with our dataset
Download our GNSS-Visual-Inertial dataset as described in [the next section](#GVINS_dataset). Then launch GVINS via:
```
roslaunch gvins visensor_f9p.launch
```
Subscribe `/gvins/gnss_enu_path` in your rviz and play the bag:
```
rosbag play sports_field.bag
```

## 4. <a name="GVINS_dataset"></a>GNSS-Visual-Inertial dataset
We published our GNSS-Visual-Inertial dataset at [rosbag_1](https://hkustconnect-my.sharepoint.com/:u:/g/personal/scaoad_connect_ust_hk/EbjRwwyRLDlOtfMVJ57MMl8Bm-jkc3rBG5HVDqvTcAAiPg?e=r5kSw8) and [rosbag_2](https://hkustconnect-my.sharepoint.com/:u:/g/personal/scaoad_connect_ust_hk/ERn2tlHnWiVDjvLSml9TgH0B6LnXLcYPd3d7toJovLRL8g?e=hRem3Y). The visual and inertial data are collected using a [VI-Sensor](https://github.com/ethz-asl/libvisensor), and the GNSS raw measurement is provided by a [u-blox ZED-F9P receiver](https://www.u-blox.com/en/product/zed-f9p-module). The RTCM stream from a 3km-away base station is fed to the GNSS receiver for RTK solution. In addition, the time synchronization between VI-Sensor and ZED-F9P is achieved via hardware trigger.


## 5. Acknowledgements
The system framework and VIO part are adapted from [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono). We use [camodocal](https://github.com/hengli/camodocal) to model the camera and [ceres](http://ceres-solver.org/) to solve the optimization problem.

## 6. Licence
The source code is released under [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html) license.