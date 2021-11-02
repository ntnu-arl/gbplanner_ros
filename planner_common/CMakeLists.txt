cmake_minimum_required(VERSION 3.0.2)
project(planner_common)

add_definitions(-std=c++14 -Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -Wno-format -Werror=return-type)

# Point collision checking:
#   0 - Orignal
#   1 - Direct TSDF/ESDF distance check (requires continuous path check during execution)
#   2 - Using interpolated tsdf distance
add_definitions(-DCOL_CHECK_METHOD=0)
# Line collision checking: 
#   0 - Orignal
#   1 - Cuboid around the edge
#   2 - Direct TSDF/ESDF distance check (requires continuous path check during execution)
#   3 - Using interpolated tsdf distance
add_definitions(-DEDGE_CHECK_METHOD=1)
# Ray casting: 
#   0 - Original
#   1 - Iterative ray casting
add_definitions(-DRAY_CAST_METHOD=1)

# Verify the values of the definitions
get_directory_property( DirDefs COMPILE_DEFINITIONS ) 
message( "COMPILE DEFINITIONS: ${DirDefs}" )

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roscpp_serialization REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/graph_manager.cpp
  src/graph.cpp
  src/params.cpp
  src/trajectory.cpp
  src/random_sampler.cpp
  src/voxblox/voxblox_common_impl.cpp
  src/voxblox/voxblox_alt_impl.cpp
  src/geofence_manager.cpp
)

cs_install()
cs_export(LIBRARIES)