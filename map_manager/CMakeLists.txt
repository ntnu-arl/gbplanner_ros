cmake_minimum_required(VERSION 3.0.2)
project(map_manager)

add_definitions(-std=c++17 -Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -Wno-format -Werror=return-type)

# Ray casting: 
#   0 - Original
#   1 - Iterative ray casting
add_definitions(-DRAY_CAST_METHOD=0)

# Verify the values of the definitions
get_directory_property( DirDefs COMPILE_DEFINITIONS ) 
message( "COMPILE DEFINITIONS: ${DirDefs}" )

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(planner_common REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/map_manager.cpp
  src/voxblox/voxblox_common_impl.cpp
  src/voxblox/voxblox_alt_impl.cpp
)

cs_install()
cs_export(LIBRARIES)