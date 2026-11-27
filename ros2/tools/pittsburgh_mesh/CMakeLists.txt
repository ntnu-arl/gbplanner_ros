cmake_minimum_required(VERSION 3.16)
project(pittsburgh_mesh_inspection LANGUAGES CXX)

# Standalone asset inspection: no ROS workspace targets or Gazebo processes.
find_package(ignition-common4 REQUIRED COMPONENTS graphics)
add_executable(check_pittsburgh_mesh ../check_pittsburgh_mesh.cpp)
target_compile_features(check_pittsburgh_mesh PRIVATE cxx_std_17)
target_compile_options(check_pittsburgh_mesh PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(check_pittsburgh_mesh PRIVATE
  ignition-common4::ignition-common4-graphics)
