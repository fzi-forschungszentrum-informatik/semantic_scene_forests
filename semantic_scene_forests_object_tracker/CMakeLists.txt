cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_object_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(semantic_scene_forests_object_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(semantic_scene_forests_utils REQUIRED)

generate_parameter_library(object_tracker_parameters
  src/object_tracker_parameters.yaml
)

# Tracker component library
add_library(object_tracker_component SHARED
  src/object_tracker_component.cpp
  src/tracked_object.cpp
  src/tracker_geometry.cpp
)
target_include_directories(object_tracker_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(object_tracker_component PUBLIC cxx_std_20)
target_link_libraries(object_tracker_component PUBLIC
  ${geometry_msgs_TARGETS}
  ${semantic_scene_forests_msgs_TARGETS}
  ${semantic_scene_forests_object_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  cv_bridge::cv_bridge
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  diagnostic_updater::diagnostic_updater
  semantic_scene_forests_utils::semantic_scene_forests_utils
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  object_tracker_parameters
)
rclcpp_components_register_nodes(object_tracker_component
  "semantic_scene_forests_object_tracker::ObjectTrackerComponent"
)

install(TARGETS
  object_tracker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bbox_transform test/test_bbox_transform.cpp)
  target_link_libraries(test_bbox_transform object_tracker_component)
  target_include_directories(test_bbox_transform PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()

ament_package()
