cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(IGN_PLUGIN_VER 1)
set(IGN_COMMON_VER 3)

find_package(ignition-rendering6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS all)

add_library(HelloWorldPlugin SHARED HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin
  PUBLIC
    ${IGNITION-RENDERING_LIBRARIES}
    ${ignition-common${IGN_COMMON_VER}_LIBRARIES}
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
)
