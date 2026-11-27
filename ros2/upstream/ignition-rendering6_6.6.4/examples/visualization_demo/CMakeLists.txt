cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(ignition-rendering-visualization-demo)

find_package(ignition-rendering6 REQUIRED)
find_package(ignition-common4 REQUIRED)

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

if (WIN32)
  find_package(FreeGLUT REQUIRED)
  set(TARGET_THIRD_PARTY_DEPENDS ${TARGET_THIRD_PARTY_DEPENDS} FreeGLUT::freeglut)
endif()

add_executable(visualization_demo Main.cc GlutWindow.cc)

target_link_libraries(visualization_demo
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
  ${TARGET_THIRD_PARTY_DEPENDS}
)
