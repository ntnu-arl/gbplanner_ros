# set compile definitions for tests
set_property(
  SOURCE Camera_TEST.cc RenderTarget.cc
  PROPERTY COMPILE_DEFINITIONS
  HAVE_OPENGL="${HAVE_OPENGL}"
)

# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
ign_get_libsources_and_unittests(sources gtest_sources)

add_subdirectory(base)

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${sources} ${gtest_sources} COMPILE_FLAGS "/wd4251")
endif()

# Create the library target.
ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

# Include the interface directories that we always need.
#ign_target_interface_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
#  ignition-math${IGN_MATH_VER})

set_property(
  SOURCE RenderEngineManager.cc
  PROPERTY COMPILE_DEFINITIONS
  IGN_RENDERING_PLUGIN_PATH="${IGN_RENDERING_PLUGIN_PATH}"
)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::requested
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::loader
)
if (UNIX AND NOT APPLE)
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE X11)
endif()

# Build the unit tests.
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

