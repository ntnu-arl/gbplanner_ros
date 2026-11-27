# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
ign_get_libsources_and_unittests(sources gtest_sources)

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${sources} ${gtest_sources} COMPILE_FLAGS "/wd4251")
endif()

set(engine_name "ogre")

ign_add_component(${engine_name} SOURCES ${sources} GET_TARGET_NAME ogre_target)

if(OGRE_VERSION VERSION_LESS 1.11.0)
  add_definitions(-DOGRE_VERSION_LT_1_11_0)
endif()
if(OGRE_VERSION VERSION_LESS 1.12.0)
  add_definitions(-DOGRE_VERSION_LT_1_12_0)
endif()

find_package(OpenGL)

set_property(
  SOURCE OgreRenderEngine.cc OgreRTShaderSystem.cc OgreGpuRays.cc OgreMaterial.cc OgreDepthCamera.cc OgreThermalCamera.cc
  PROPERTY COMPILE_DEFINITIONS
  IGN_RENDERING_RESOURCE_PATH="${IGN_RENDERING_RESOURCE_PATH}"
  OGRE_RESOURCE_PATH="${OGRE_RESOURCE_PATH}"
)

target_link_libraries(${ogre_target}
  PUBLIC
    ${ignition-common${IGN_COMMON_VER}_LIBRARIES}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
    ${OPENGL_LIBRARIES}
    IgnOGRE::IgnOGRE
    )

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources} LIB_DEPS ${ogre_target})

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${ogre_target} DESTINATION ${IGNITION_RENDERING_ENGINE_INSTALL_DIR})

if(WIN32)
  # tests needs .dll in the same directory
  add_custom_command(TARGET ${ogre_target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:${ogre_target}> ${CMAKE_CURRENT_BINARY_DIR})
endif()

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

add_subdirectory(media)
