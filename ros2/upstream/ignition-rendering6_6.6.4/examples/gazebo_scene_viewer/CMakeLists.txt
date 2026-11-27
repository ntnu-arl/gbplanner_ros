cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(ignition-rendering-gazebo-scene-viewer)

find_package(ignition-rendering6 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(gazebo REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})

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
  set(STD_CXX_FS_LIBRARIES "stdc++fs")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

add_executable(gazebo_scene_viewer
  CameraWindow.cc
  GazeboDemo.cc
  SceneManager.cc
)

target_link_libraries(gazebo_scene_viewer
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
  ${STD_CXX_FS_LIBRARIES}
)

add_executable(gazebo_scene_viewer2_demo
  CameraWindow.cc
  GazeboWorldDemo.cc
  SceneManager.cc
)

target_link_libraries(gazebo_scene_viewer2_demo
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
  ${STD_CXX_FS_LIBRARIES}
)
