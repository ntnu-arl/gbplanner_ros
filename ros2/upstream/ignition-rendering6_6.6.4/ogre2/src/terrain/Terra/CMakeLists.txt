
project(terra)

file( GLOB_RECURSE TERRA_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/include/*.h
  ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME} STATIC ${TERRA_SOURCES})

if(IGN_ADD_fPIC_TO_LIBRARIES AND NOT _ign_add_library_INTERFACE)
  target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)
endif()

# disable all warnings for Terra
if (UNIX)
  target_compile_options(${PROJECT_NAME} PRIVATE
      -Wno-unused-parameter
      -Wno-float-equal
      -Wno-sign-compare
      -Wno-strict-aliasing)
elseif (MSVC)
  set_source_files_properties(${TERRA_SOURCES} COMPILE_FLAGS "/wd4244")
endif()

add_definitions(-DOGRE_IGNORE_UNKNOWN_DEBUG)

#target_compile_definitions(${PROJECT_NAME} PUBLIC
#  $<$<CONFIG:Debug>:DEBUG=1 _DEBUG=1>)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    # Hlms files inside Hlms/Pbs do not have #include <Hlms/Pbs/...> thus
    # we must add this one manually for this to build correctly
    ${OGRE2_INCLUDE}/Hlms/Pbs
    ${OGRE2_INCLUDE}/Hlms/Common
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PRIVATE IgnOGRE2::IgnOGRE2)
