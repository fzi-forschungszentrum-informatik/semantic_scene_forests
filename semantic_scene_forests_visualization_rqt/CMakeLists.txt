cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_visualization_rqt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(semantic_scene_forests REQUIRED)
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(semantic_scene_forests_object_msgs REQUIRED)
find_package(semantic_scene_forests_planning_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(fmt REQUIRED)

find_package(Qt6 QUIET COMPONENTS Core Widgets Gui)
if(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
  set(QT_VERSION_MAJOR 5)
endif()

cmake_language(CALL qt${QT_VERSION_MAJOR}_wrap_cpp MOC_FILES
  include/semantic_scene_forests_visualization_rqt/scene_graph_plugin.hpp
  include/semantic_scene_forests_visualization_rqt/graph_view.hpp
  include/semantic_scene_forests_visualization_rqt/image_panel.hpp
  include/semantic_scene_forests_visualization_rqt/side_panel.hpp
)

add_library(${PROJECT_NAME} SHARED
  src/scene_graph_plugin.cpp
  src/graph_view.cpp
  src/node_item.cpp
  src/edge_item.cpp
  src/image_panel.cpp
  src/side_panel.cpp
  src/image_utils.cpp
  src/color_palette.cpp
  src/plugin_registration.cpp
  ${MOC_FILES}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  semantic_scene_forests::semantic_scene_forests
  ${semantic_scene_forests_msgs_TARGETS}
  ${semantic_scene_forests_object_msgs_TARGETS}
  ${semantic_scene_forests_planning_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rqt_gui_cpp::rqt_gui_cpp
  pluginlib::pluginlib
  fmt::fmt
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Gui
  Qt${QT_VERSION_MAJOR}::Widgets
)

pluginlib_export_plugin_description_file(rqt_gui plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
