cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch etc
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/launch_semantic_scene_forests.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
