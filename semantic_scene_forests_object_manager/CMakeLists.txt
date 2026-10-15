cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_object_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(semantic_scene_forests_object_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(semantic_scene_forests_utils REQUIRED)

generate_parameter_library(object_manager_parameters
  src/object_manager_parameters.yaml
)

add_library(object_manager_component SHARED
  src/managed_object.cpp
  src/object_manager.cpp
  src/object_manager_node.cpp
)

target_include_directories(object_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(object_manager_component PUBLIC cxx_std_20)

target_link_libraries(object_manager_component PUBLIC
  ${geometry_msgs_TARGETS}
  ${semantic_scene_forests_msgs_TARGETS}
  ${semantic_scene_forests_object_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  diagnostic_updater::diagnostic_updater
  semantic_scene_forests_utils::semantic_scene_forests_utils
  object_manager_parameters
)

rclcpp_components_register_nodes(object_manager_component
  "semantic_scene_forests_object_manager::ObjectManagerNode"
)

install(TARGETS
  object_manager_component
  object_manager_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_object_manager test/test_object_manager.cpp)
  target_link_libraries(test_object_manager object_manager_component)
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  semantic_scene_forests_msgs
  semantic_scene_forests_object_msgs
  semantic_scene_forests_utils
  sensor_msgs
  std_msgs
  vision_msgs
)

ament_package()
