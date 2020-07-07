cmake_minimum_required(VERSION 2.8.3)
project(kdtree)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/kdtree.c
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
)

##########
# EXPORT #
##########
cs_install()
cs_export()
