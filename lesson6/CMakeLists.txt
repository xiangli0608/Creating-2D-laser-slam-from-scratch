cmake_minimum_required(VERSION 3.0.2)
project(lesson6)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE Debug) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# lib karto 
add_subdirectory(lib/open_karto)

# lib sparse_bundle_adjustment 
add_subdirectory(lib/sparse_bundle_adjustment)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
geometry_msgs
nav_msgs
roscpp
sensor_msgs
tf
open_karto
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Find Eigen3 (from http://wiki.ros.org/jade/Migration)
find_package(Eigen3 REQUIRED)

# Ceres
find_package(Ceres REQUIRED)

# GTSAM
find_package(GTSAM REQUIRED)

# g2o  
find_package(G2O REQUIRED)   

# suitesparse   
find_package(SUITESPARSE REQUIRED) 

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES lesson6
CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
lib/sparse_bundle_adjustment/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}

  ${G2O_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
)

## Karto SLAM
add_executable(${PROJECT_NAME}_karto_slam_node
  src/karto_slam.cc
  src/spa_solver/spa_solver.cc
  src/ceres_solver/ceres_solver.cc
  src/gtsam_solver/gtsam_solver.cc
  src/g2o_solver/g2o_solver.cc
)

add_dependencies(${PROJECT_NAME}_karto_slam_node
${catkin_EXPORTED_TARGETS}
sba
)

target_link_libraries(${PROJECT_NAME}_karto_slam_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  sba
  ${CERES_LIBRARIES}
  gtsam

  ${G2O_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)
