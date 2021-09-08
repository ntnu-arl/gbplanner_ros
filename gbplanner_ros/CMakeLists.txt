cmake_minimum_required(VERSION 2.8.3)
project(gbplanner_ros)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export(LIBRARIES)
