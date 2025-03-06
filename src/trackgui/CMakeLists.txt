cmake_minimum_required(VERSION 3.5)
project(trackgui)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
)

ament_package()
