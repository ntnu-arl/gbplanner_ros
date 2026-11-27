set(TEST_TYPE "INTEGRATION")

set(tests
  gpu_rays.cc
  boundingbox_camera.cc
  depth_camera.cc
  camera.cc
  render_pass.cc
  shadows.cc
  scene.cc
  segmentation_camera.cc
  sky.cc
  thermal_camera.cc
  lidar_visual.cc
  heightmap.cc
)

if (APPLE)
  list(REMOVE_ITEM tests gpu_rays.cc)
  list(REMOVE_ITEM tests camera.cc)
  list(REMOVE_ITEM tests depth_camera.cc)
  list(REMOVE_ITEM tests render_pass.cc)
  list(REMOVE_ITEM tests shadows.cc)
  list(REMOVE_ITEM tests scene.cc)
  list(REMOVE_ITEM tests thermal_camera.cc)
  list(REMOVE_ITEM tests lidar_visual.cc)
endif()

link_directories(${PROJECT_BINARY_DIR}/test)

# Test symbols having the right name on linux only
if (UNIX AND NOT APPLE)
  configure_file(all_symbols_have_version.bash.in ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash @ONLY)
  add_test(NAME INTEGRATION_versioned_symbols
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>)

  foreach(comp ${RENDERING_COMPONENTS})
   set(comp_target_name "${PROJECT_LIBRARY_TARGET_NAME}-${comp}")
   add_test(NAME "INTEGRATION_versioned_symbols_${comp}"
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash $<TARGET_FILE:${comp_target_name}>)
  endforeach()
endif()

ign_build_tests(TYPE INTEGRATION SOURCES ${tests})
