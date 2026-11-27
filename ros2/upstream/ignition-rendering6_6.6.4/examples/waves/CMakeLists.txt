cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(ignition-rendering-waves)

include_directories(SYSTEM
  ${PROJECT_BINARY_DIR}
)

find_package(ignition-rendering6 REQUIRED)
find_package(ignition-common4 REQUIRED)

set(TARGET_THIRD_PARTY_DEPENDS "")

if (APPLE OR UNIX)
  find_package(GLUT REQUIRED)
  include_directories(SYSTEM ${GLUT_INCLUDE_DIRS})
  link_directories(${GLUT_LIBRARY_DIRS})

  find_package(OpenGL REQUIRED)
  include_directories(SYSTEM ${OpenGL_INCLUDE_DIRS})
  link_directories(${OpenGL_LIBRARY_DIRS})
  set(TARGET_THIRD_PARTY_DEPENDS
    ${TARGET_THIRD_PARTY_DEPENDS}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
  )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
endif()

if (NOT APPLE)
  find_package(GLEW REQUIRED)
  if (WIN32)
    set(TARGET_THIRD_PARTY_DEPENDS
      ${TARGET_THIRD_PARTY_DEPENDS}
      GLEW::glew
    )
  else ()
    set(TARGET_THIRD_PARTY_DEPENDS
      ${TARGET_THIRD_PARTY_DEPENDS}
      GLEW
    )
  endif()
endif()

configure_file (example_config.hh.in ${PROJECT_BINARY_DIR}/example_config.hh)

if (WIN32)
  find_package(FreeGLUT REQUIRED)
  set(TARGET_THIRD_PARTY_DEPENDS ${TARGET_THIRD_PARTY_DEPENDS} FreeGLUT::freeglut)
endif()

add_executable(waves Main.cc GlutWindow.cc)

target_link_libraries(waves
  ${IGNITION-RENDERING_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${TARGET_THIRD_PARTY_DEPENDS}
)

if (WIN32)
  set_target_properties(waves
    PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
      RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}
      RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}
      RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_BINARY_DIR}
      RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${PROJECT_BINARY_DIR}
  )
endif()

add_custom_command(TARGET waves POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_SOURCE_DIR}/media
  $<TARGET_FILE_DIR:waves>/media)
