cmake_minimum_required(VERSION 2.8.3)
project(gvins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    rosbag
    message_generation
    gnss_comm
)

add_message_files(
  DIRECTORY msg
  FILES LocalSensorExternalTrigger.msg
)
generate_messages(DEPENDENCIES std_msgs)

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(${PROJECT_NAME}
    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/factor/gnss_psr_dopp_factor.cpp
    src/factor/gnss_dt_ddt_factor.cpp
    src/factor/gnss_dt_anchor_factor.cpp
    src/factor/gnss_ddt_smooth_factor.cpp
    src/factor/pos_vel_factor.cpp
    src/factor/pose_anchor_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/initial/gnss_vi_initializer.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
