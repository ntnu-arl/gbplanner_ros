# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
ign_get_libsources_and_unittests(sources gtest_sources)

include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

# Needed by add_custom_target: ${optix_target}-ptx
include_directories("${PROJECT_SOURCE_DIR}/optix/include")
include_directories(SYSTEM ${OptiX_INCLUDE_DIRS})

set(engine_name optix)

ign_add_component(${engine_name} SOURCES ${sources} GET_TARGET_NAME optix_target)

set(cuda_sources
  OptixBox.cu
  OptixCone.cu
  OptixCylinder.cu
  OptixCamera.cu
  OptixErrorProgram.cu
  OptixMaterial.cu
  OptixMissProgram.cu
  OptixMesh.cu
  # OptixPlane.cu
  # OptixBackgroundColor.cu
  OptixSphere.cu
)

set_property(
  SOURCE OptixRenderEngine.cc
  PROPERTY COMPILE_DEFINITIONS
  IGN_RENDERING_RESOURCE_PATH="${IGN_RENDERING_RESOURCE_PATH}"
)

target_link_libraries(${optix_target}
  PUBLIC
    ${ignition-common${IGN_COMMON_VER}_LIBRARIES}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
    optix::optix
    optix::optixu
    optix::optix_prime)

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${optix_target} DESTINATION ${IGNITION_RENDERING_ENGINE_INSTALL_DIR})

set (versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_LOWER}-${engine_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
set (unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${engine_name}${CMAKE_SHARED_LIBRARY_SUFFIX})

if (WIN32)
  # disable MSVC inherit via dominance warning
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4250")
  INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy
  ${IGNITION_RENDERING_ENGINE_INSTALL_DIR}\/${versioned} 
  ${IGNITION_RENDERING_ENGINE_INSTALL_DIR}\/${unversioned})")
else()
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
  INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${IGNITION_RENDERING_ENGINE_INSTALL_DIR})
endif()

if("${CUDA_VERSION}" VERSION_LESS "9")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_20,code=sm_20)
endif()

cuda_compile_ptx(ptx_files ${cuda_sources})

add_custom_target(${optix_target}-ptx ALL
                  DEPENDS ${ptx_files} ${cuda_sources}
                  SOURCES ${cuda_sources})

set(ptx_dir "${IGN_RENDERING_RESOURCE_PATH}/optix")
install(FILES ${ptx_files} DESTINATION ${ptx_dir})


