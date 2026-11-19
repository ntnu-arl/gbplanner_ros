cmake_minimum_required(VERSION 3.8)
project(kdtree C CXX)
# ROS 2 (ament) build of the GBPlanner port; sources are the ROS 1 ones with the ROS API ported.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -Wno-format -Wno-unused-parameter -Wno-reorder -Werror=return-type)
find_package(ament_cmake REQUIRED)

set(EXTRA_LIBS "")

add_library(${PROJECT_NAME} SHARED src/kdtree.c)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${EXTRA_LIBS})

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY include/ DESTINATION include OPTIONAL)
ament_export_include_directories(include)

ament_package()
