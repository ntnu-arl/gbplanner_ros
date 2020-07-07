cmake_minimum_required(VERSION 2.8.3)
project(planner_gazebo_sim)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})