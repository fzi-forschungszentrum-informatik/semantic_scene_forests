cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SceneGraph.msg"
  "msg/Edge.msg"
  "msg/ObjectNode.msg"
  "msg/ConceptNode.msg"
  "action/UpdateSceneGraph.action"
  DEPENDENCIES std_msgs vision_msgs sensor_msgs
)

ament_package()
