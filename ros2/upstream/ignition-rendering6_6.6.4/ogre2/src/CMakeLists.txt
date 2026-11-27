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

set(engine_name "ogre2")

ign_add_component(${engine_name} SOURCES ${sources} GET_TARGET_NAME ogre2_target)

find_package(OpenGL)

add_definitions(-DOGRE_IGNORE_UNKNOWN_DEBUG)

set_property(
  SOURCE Ogre2RenderEngine.cc
  PROPERTY COMPILE_DEFINITIONS
  IGN_RENDERING_RESOURCE_PATH="${IGN_RENDERING_RESOURCE_PATH}"
  OGRE2_RESOURCE_PATH="${OGRE2_RESOURCE_PATH}"
  OGRE2_VERSION="${OGRE2_VERSION}"
)

target_include_directories(${ogre2_target}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${ogre2_target}
  PUBLIC
    ${ignition-common${IGN_COMMON_VER}_LIBRARIES}
    ignition-math${IGN_MATH_VER}::eigen3
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
    ${OPENGL_LIBRARIES}
    terra
    IgnOGRE2::IgnOGRE2)

set (versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_LOWER}-${engine_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
set (unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${engine_name}${CMAKE_SHARED_LIBRARY_SUFFIX})

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${ogre2_target} DESTINATION ${IGNITION_RENDERING_ENGINE_INSTALL_DIR})

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

if (NOT (APPLE OR WIN32))
  option(USE_GLX "Use GLX to create OpenGL contexts." ON)
else()
  option(USE_GLX "Use GLX to create OpenGL contexts." OFF)
endif()
if (USE_GLX)
  add_definitions(-DHAVE_GLX=1)
endif()

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources} LIB_DEPS ${ogre2_target})

install(DIRECTORY "media"  DESTINATION ${IGN_RENDERING_RESOURCE_PATH}/ogre2)
