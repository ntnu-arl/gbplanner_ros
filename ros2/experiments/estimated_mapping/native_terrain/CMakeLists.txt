cmake_minimum_required(VERSION 3.16)
project(estimated_mapping_native_terrain LANGUAGES CXX)

# Consume the installed planning core without rebuilding production nodes.
find_package(gbplanner_local REQUIRED)
add_executable(native_terrain_validate native_terrain_validate.cpp)
target_compile_features(native_terrain_validate PRIVATE cxx_std_17)
target_compile_options(native_terrain_validate PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(native_terrain_validate PRIVATE gbplanner_local::gbplanner_local)
