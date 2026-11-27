cmake_minimum_required(VERSION 3.16)
project(voxblox_core)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)
find_package(Threads REQUIRED)
# Replace the catkin build while retaining byte-identical vendor sources.
# The corrections below are applied only to build-tree copies.
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  vendor/voxblox/proto/voxblox/Block.proto vendor/voxblox/proto/voxblox/Layer.proto)
add_library(voxblox_proto SHARED ${PROTO_SRCS})
target_include_directories(voxblox_proto PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> $<INSTALL_INTERFACE:include>)
target_link_libraries(voxblox_proto PUBLIC protobuf::libprotobuf)
file(GLOB_RECURSE VOXBLOX_SOURCES CONFIGURE_DEPENDS vendor/voxblox/src/*.cc)

# The pinned upstream RayCaster tests abs(direction) < 0.0 on all six
# zero-direction guards. For axis-aligned rays this permits division by zero
# and can repeatedly visit the starting voxel instead of traversing the ray.
# Restore the intended zero case; keep the upstream 2.0 sentinel and step order.
set(VOXBLOX_RAY_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/voxblox/src/integrator/integrator_utils.cc")
file(SHA256 "${VOXBLOX_RAY_SOURCE}" VOXBLOX_RAY_SOURCE_SHA256)
if(NOT VOXBLOX_RAY_SOURCE_SHA256 STREQUAL
    "e017d889c087a28ef8b17855e563738f322223a2e07e9343bf96931a3665a128")
  message(FATAL_ERROR "Pinned RayCaster source changed; review the zero-direction build patch")
endif()
file(READ "${VOXBLOX_RAY_SOURCE}" VOXBLOX_RAY_SOURCE_TEXT)
string(REGEX MATCHALL [=[std::abs\(ray_scaled\.[xyz]\(\)\) < 0\.0]=]
  VOXBLOX_RAY_ZERO_GUARDS "${VOXBLOX_RAY_SOURCE_TEXT}")
list(LENGTH VOXBLOX_RAY_ZERO_GUARDS VOXBLOX_RAY_ZERO_GUARD_COUNT)
if(NOT VOXBLOX_RAY_ZERO_GUARD_COUNT EQUAL 6)
  message(FATAL_ERROR "Expected exactly six RayCaster zero-direction guards")
endif()
foreach(axis x y z)
  string(REPLACE "std::abs(ray_scaled.${axis}()) < 0.0"
    "std::abs(ray_scaled.${axis}()) == 0.0"
    VOXBLOX_RAY_SOURCE_TEXT "${VOXBLOX_RAY_SOURCE_TEXT}")
endforeach()
set(VOXBLOX_PATCHED_RAY_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/patched/integrator_utils.cc")
file(GENERATE OUTPUT "${VOXBLOX_PATCHED_RAY_SOURCE}" CONTENT "${VOXBLOX_RAY_SOURCE_TEXT}")
set_source_files_properties("${VOXBLOX_PATCHED_RAY_SOURCE}" PROPERTIES GENERATED TRUE)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${VOXBLOX_RAY_SOURCE}")
list(REMOVE_ITEM VOXBLOX_SOURCES "${VOXBLOX_RAY_SOURCE}")
list(APPEND VOXBLOX_SOURCES "${VOXBLOX_PATCHED_RAY_SOURCE}")

# Multiple integration workers can visit the same block. Its updated() flag is
# a plain bitset, so even concurrent set-all operations need synchronization.
# Mark only when a worker's local ray cache changes block; readers/clearers run
# after integratePointCloud joins the workers. Reuse the existing hashed mutex
# array, with a block-wide key rather than a voxel key. Allocation releases its
# temporary-block lock before this lock, which is released before voxel fusion.
# This leaves the original ray selection, numerical updates and vendor ABI alone.
set(VOXBLOX_TSDF_SOURCE
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/voxblox/src/integrator/tsdf_integrator.cc")
file(SHA256 "${VOXBLOX_TSDF_SOURCE}" VOXBLOX_TSDF_SOURCE_SHA256)
if(NOT VOXBLOX_TSDF_SOURCE_SHA256 STREQUAL
    "d979df3348ffdf0acb46df2e041b940e7d1d7774d87fc66d90cb8b6ed3d16e08")
  message(FATAL_ERROR "Pinned TSDF source changed; review the block-update synchronization patch")
endif()
file(READ "${VOXBLOX_TSDF_SOURCE}" VOXBLOX_TSDF_SOURCE_TEXT)
set(VOXBLOX_TSDF_CACHE_OLD [=[  if ((block_idx != *last_block_idx) || (*last_block == nullptr)) {]=])
set(VOXBLOX_TSDF_CACHE_NEW [=[  // A new ray has a null block and an uninitialized cached block index.
  const bool block_changed =
      (*last_block == nullptr) || (block_idx != *last_block_idx);
  if (block_changed) {]=])
set(VOXBLOX_TSDF_MARK_OLD [=[  (*last_block)->updated().set();]=])
set(VOXBLOX_TSDF_MARK_NEW [=[  if (block_changed) {
    // BlockIndex is int32; the existing mutex array takes int64 GlobalIndex.
    // No temporary-block or voxel-update lock is held here, including when
    // their hash buckets coincide. Consumers inspect flags after worker join.
    const GlobalIndex block_lock_index = block_idx.cast<LongIndexElement>();
    std::lock_guard<std::mutex> block_lock(mutexes_.get(block_lock_index));
    (*last_block)->updated().set();
  }]=])
foreach(VOXBLOX_TSDF_PATCH IN ITEMS CACHE MARK)
  set(VOXBLOX_TSDF_OLD "${VOXBLOX_TSDF_${VOXBLOX_TSDF_PATCH}_OLD}")
  set(VOXBLOX_TSDF_NEW "${VOXBLOX_TSDF_${VOXBLOX_TSDF_PATCH}_NEW}")
  string(FIND "${VOXBLOX_TSDF_SOURCE_TEXT}" "${VOXBLOX_TSDF_OLD}" VOXBLOX_TSDF_MATCH)
  if(VOXBLOX_TSDF_MATCH LESS 0)
    message(FATAL_ERROR "Missing pinned TSDF ${VOXBLOX_TSDF_PATCH} patch site")
  endif()
  math(EXPR VOXBLOX_TSDF_AFTER_MATCH "${VOXBLOX_TSDF_MATCH} + 1")
  string(SUBSTRING "${VOXBLOX_TSDF_SOURCE_TEXT}" ${VOXBLOX_TSDF_AFTER_MATCH} -1 VOXBLOX_TSDF_TAIL)
  string(FIND "${VOXBLOX_TSDF_TAIL}" "${VOXBLOX_TSDF_OLD}" VOXBLOX_TSDF_DUPLICATE)
  if(NOT VOXBLOX_TSDF_DUPLICATE EQUAL -1)
    message(FATAL_ERROR "Expected exactly one pinned TSDF ${VOXBLOX_TSDF_PATCH} patch site")
  endif()
  string(REPLACE "${VOXBLOX_TSDF_OLD}" "${VOXBLOX_TSDF_NEW}"
    VOXBLOX_TSDF_SOURCE_TEXT "${VOXBLOX_TSDF_SOURCE_TEXT}")
endforeach()
set(VOXBLOX_PATCHED_TSDF_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/patched/tsdf_integrator.cc")
file(GENERATE OUTPUT "${VOXBLOX_PATCHED_TSDF_SOURCE}" CONTENT "${VOXBLOX_TSDF_SOURCE_TEXT}")
set_source_files_properties("${VOXBLOX_PATCHED_TSDF_SOURCE}" PROPERTIES GENERATED TRUE)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${VOXBLOX_TSDF_SOURCE}")
list(REMOVE_ITEM VOXBLOX_SOURCES "${VOXBLOX_TSDF_SOURCE}")
list(APPEND VOXBLOX_SOURCES "${VOXBLOX_PATCHED_TSDF_SOURCE}")

add_library(voxblox_core SHARED ${VOXBLOX_SOURCES})
target_include_directories(voxblox_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/voxblox/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/minkindr/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(voxblox_core PUBLIC voxblox_proto Eigen3::Eigen glog::glog Threads::Threads)
install(TARGETS voxblox_core voxblox_proto EXPORT voxbloxTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY vendor/voxblox/include/ vendor/minkindr/include/ DESTINATION include)
install(FILES ${PROTO_HDRS} DESTINATION include)
install(FILES LICENSE provenance.json DESTINATION share/${PROJECT_NAME})
install(FILES vendor/minkindr/LICENSE DESTINATION share/${PROJECT_NAME}/minkindr)
ament_export_targets(voxbloxTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Protobuf glog Threads)
ament_package()
