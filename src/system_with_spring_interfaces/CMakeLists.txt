cmake_minimum_required(VERSION 3.8)
project(system_with_spring_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
# ...

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpringSystem.msg"
  "msg/PendulumCoord.msg"
  "msg/SpringCoord.msg"
 )

 ament_export_dependencies(rosidl_default_runtime)

 ament_package()