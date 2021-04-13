FROM ros:kinetic-perception

ENV EIGEN_VERSION="3.3.3"
ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/root/catkin_ws

      # set up thread number for building
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      git \
      cmake \
      libatlas-base-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf && \
      rm -rf /var/lib/apt/lists/* && \
      # install eigen
      git clone https://gitlab.com/libeigen/eigen.git && \
      cd eigen && \
      git checkout tags/${EIGEN_VERSION} && \
      mkdir build && cd build && \
      cmake .. && make install && \
      cd ../.. && rm -rf eigen && \
      # Build and install Ceres
      git clone https://ceres-solver.googlesource.com/ceres-solver && \
      cd ceres-solver && \
      git checkout tags/${CERES_VERSION} && \
      mkdir build && cd build && \
      cmake .. && \
      make -j$(USE_PROC) install && \
      cd ../.. && rm -rf ceres-solver && \
      # create GVINS directory
      mkdir -p $CATKIN_WS/src/GVINS/ && \
      cd $CATKIN_WS/src && \
      # clone gnss_comm repo
      git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git

# Copy the local replica of GVINS
COPY ./ $CATKIN_WS/src/GVINS/
# comment the above line and use the following command if you only have this dockerfile
# RUN git clone https://github.com/HKUST-Aerial-Robotics/GVINS.git

# Build GVINS
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    sed -i '/exec "$@"/i \
            source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh
