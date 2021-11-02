cmake_minimum_required(VERSION 3.0.2)
project(gbplanner)

add_definitions(-std=c++14 -Wall -Wno-sign-compare -Wno-catch-value -Wno-switch -Wno-maybe-uninitialized -Werror=return-type)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(adaptive_obb REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${planner_common_INCLUDE_DIRS}
)

cs_add_library(
  ${PROJECT_NAME}
  src/gbplanner.cpp
  src/rrg.cpp
  src/gbplanner_rviz.cpp
)

cs_add_executable(gbplanner_node src/gbplanner_ros_node.cpp)
target_link_libraries(gbplanner_node ${PROJECT_NAME} ${planner_common_LIBRARIES})

cs_install()
cs_export(LIBRARIES)
