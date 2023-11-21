cmake_minimum_required(VERSION 3.8)
project(double_pendulum_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
# ...

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DoublePendulum.msg"
  "msg/CoordX.msg"
  "msg/CoordY.msg"
 )

 ament_export_dependencies(rosidl_default_runtime)

 ament_package()