cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(ignition-rendering-heightmap)

find_package(ignition-rendering6 REQUIRED)
find_package(ignition-common4 REQUIRED)

include_directories(SYSTEM
  ${PROJECT_BINARY_DIR}
)

find_package(GLUT REQUIRED)
include_directories(SYSTEM ${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})

find_package(OpenGL REQUIRED)
include_directories(SYSTEM ${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})

if (NOT APPLE)
  find_package(GLEW REQUIRED)
  include_directories(SYSTEM ${GLEW_INCLUDE_DIRS})
  link_directories(${GLEW_LIBRARY_DIRS})
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

configure_file (example_config.hh.in ${PROJECT_BINARY_DIR}/example_config.hh)

add_executable(heightmap Main.cc GlutWindow.cc)

target_link_libraries(heightmap
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
)

add_custom_command(TARGET heightmap POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_SOURCE_DIR}/media
  $<TARGET_FILE_DIR:heightmap>/media)
