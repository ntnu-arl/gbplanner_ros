cmake_minimum_required(VERSION 3.0.2)
project(pci_general)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${Eigen_INCLUDE_DIRS}
)

cs_add_library(
  ${PROJECT_NAME}
  src/pci_general.cpp)

cs_add_executable(pci_general_ros_node src/pci_general_ros_node.cpp)
target_link_libraries(pci_general_ros_node ${PROJECT_NAME})

cs_install()
cs_export(LIBRARIES)
