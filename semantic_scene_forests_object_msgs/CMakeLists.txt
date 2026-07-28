cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_object_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManagedObject.msg"
  "msg/ManagedObjectArray.msg"
  "msg/ObjectCandidate.msg"
  "msg/ObjectVisuals.msg"
  "srv/GetObjects.srv"
  "srv/GetObjectVisuals.srv"
  "srv/BlockObject.srv"
  "srv/EditObject.srv"
  "srv/UnblockObject.srv"
  DEPENDENCIES builtin_interfaces std_msgs vision_msgs sensor_msgs geometry_msgs
)

ament_package()
