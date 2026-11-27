cmake_minimum_required(VERSION 3.16)
project(gbplanner_rmf_map_parse_benchmark LANGUAGES CXX)

find_package(gbplanner_local REQUIRED)
find_package(OpenSSL REQUIRED)
add_executable(benchmark_rmf_map_parse ../benchmark_rmf_map_parse.cpp)
target_compile_features(benchmark_rmf_map_parse PRIVATE cxx_std_17)
target_compile_options(benchmark_rmf_map_parse PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(benchmark_rmf_map_parse PRIVATE gbplanner_local::gbplanner_local OpenSSL::Crypto)

# A baseline keeps its original public layout and shared object even after the
# production workspace is rebuilt with a different SparseVoxelMap private ABI.
set(RMF_FROZEN_DIR "" CACHE PATH "Frozen baseline include/ and lib/ directory")
if(RMF_FROZEN_DIR)
  target_include_directories(benchmark_rmf_map_parse BEFORE PRIVATE "${RMF_FROZEN_DIR}/include")
  set_target_properties(gbplanner_local::gbplanner_local PROPERTIES
    IMPORTED_LOCATION "${RMF_FROZEN_DIR}/lib/libgbplanner_local.so"
    IMPORTED_LOCATION_RELEASE "${RMF_FROZEN_DIR}/lib/libgbplanner_local.so")
endif()
