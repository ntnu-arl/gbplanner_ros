cmake_minimum_required(VERSION 3.16)
project(gbplanner_validation_snapshot_tool LANGUAGES CXX)

# Standalone consumer: use the existing production package and its exported
# dependencies without rebuilding planner nodes, simulation, or regression tests.
find_package(gbplanner_local REQUIRED)

add_executable(diagnose_validation_snapshot ../diagnose_validation_snapshot.cpp)
target_compile_features(diagnose_validation_snapshot PRIVATE cxx_std_17)
target_compile_options(diagnose_validation_snapshot PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(diagnose_validation_snapshot PRIVATE gbplanner_local::gbplanner_local)
