cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_visualization_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(semantic_scene_forests_object_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Qt6 QUIET COMPONENTS Core Widgets)
if(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
  set(QT_VERSION_MAJOR 5)
endif()

cmake_language(
  CALL qt${QT_VERSION_MAJOR}_wrap_cpp MOC_FILES
  include/semantic_scene_forests_visualization_rviz/scene_graph_display.hpp
  include/semantic_scene_forests_visualization_rviz/image_viewer_dialog.hpp
)

add_library(${PROJECT_NAME} SHARED
  src/scene_graph_display.cpp
  src/plugin_registration.cpp
  src/scene_graph_object_selection_handler.cpp
  src/image_viewer_dialog.cpp
  ${MOC_FILES}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${semantic_scene_forests_msgs_TARGETS}
  ${semantic_scene_forests_object_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp_action::rclcpp_action
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  pluginlib::pluginlib
  Qt${QT_VERSION_MAJOR}::Widgets
  Qt${QT_VERSION_MAJOR}::Core
)

pluginlib_export_plugin_description_file(
  rviz_common semantic_scene_forests_display_plugin.xml
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES semantic_scene_forests_display_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
