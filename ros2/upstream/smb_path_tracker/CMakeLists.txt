cmake_minimum_required(VERSION 2.8.3)
project(smb_path_tracker)

add_definitions(-std=c++14 -Wall -Wno-sign-compare -Werror=return-type)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

cs_add_library(
  ${PROJECT_NAME}
  src/pure_pursuit.cpp)

cs_add_executable(smb_path_tracker_ros_node src/path_tracker_node.cpp)
target_link_libraries(smb_path_tracker_ros_node ${PROJECT_NAME})

cs_add_executable(dummy_path_tracker_client_ros_node src/dummy_client.cpp)
target_link_libraries(dummy_path_tracker_client_ros_node ${PROJECT_NAME})

cs_install()
cs_export(LIBRARIES)
