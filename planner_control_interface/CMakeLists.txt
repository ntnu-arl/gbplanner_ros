cmake_minimum_required(VERSION 2.8.3)
project(planner_control_interface)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${Eigen_INCLUDE_DIRS}
)

cs_add_library(
  ${PROJECT_NAME}
  src/planner_control_interface.cpp)

cs_install()
cs_export(LIBRARIES)
