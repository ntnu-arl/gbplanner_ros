cmake_minimum_required(VERSION 3.16)
project(voxblox_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Block.msg"
  "msg/Layer.msg"
  "msg/Mesh.msg"
  "msg/MeshBlock.msg"
  DEPENDENCIES std_msgs)

install(FILES LICENSE SOURCE.json DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
