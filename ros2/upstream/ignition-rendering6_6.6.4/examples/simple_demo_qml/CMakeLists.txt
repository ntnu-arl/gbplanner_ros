cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(ignition-rendering-simple-demo-qml)

#------------------------------------------------------------------------
# Compile as C++14

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#------------------------------------------------------------------------
# Find ignition-cmake
find_package(ignition-cmake2 2.3 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

#------------------------------------------------------------------------
# Find ign-rendering
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-common4 REQUIRED)

#======================================
# Find Qt
find_package(Qt5 REQUIRED COMPONENTS
    Core
    Gui
    Qml
    Quick
    QuickControls2
    Widgets
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Qml_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${Qt5QuickControls2_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

add_definitions(
    ${Qt5Widgets_DEFINITIONS}
    ${Qt5Qml_DEFINITIONS}
    ${${Qt5Quick_DEFINITIONS}}
)

#======================================
# Configure Qt

qt5_add_resources(QT_RESOURCES
    Main.qrc
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#======================================
# Build

add_executable(simple_demo_qml
    Main.cc
    IgnitionRenderer.hh
    IgnitionRenderer.cc
    ThreadRenderer.h
    ThreadRenderer.cpp
    ${QT_RESOURCES}
)

target_include_directories(simple_demo_qml PUBLIC
    ${IGNITION-RENDERING_INCLUDE_DIRS}
    ${IGNITION-COMMON_INCLUDE_DIRS}
    ${OGRE2_INCLUDE_DIRS}
    ${OpenGL_INCLUDE_DIRS}
)

target_link_libraries(simple_demo_qml PUBLIC
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-RENDERING_LIBRARIES}
    ${OGRE2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Qml
    Qt5::Quick
)

set_target_properties(simple_demo_qml
    PROPERTIES
        QT_QML_MODULE_VERSION 1.0
        QT_QML_MODULE_URI IgnitionRendering
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
