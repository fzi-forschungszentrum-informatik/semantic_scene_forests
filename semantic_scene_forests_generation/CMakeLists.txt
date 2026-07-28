cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core ROS2 / ament
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Messages / services / actions
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(semantic_scene_forests_object_msgs REQUIRED)
find_package(semantic_scene_forests REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Third-party
find_package(oai_api REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(semantic_scene_forests_utils REQUIRED)

# Minja fetches at configure time and is pinned to the misc_ws commit.
include(FetchContent)

# Prevent minja from re-creating nlohmann_json targets provided by find_package.
file(WRITE "${CMAKE_BINARY_DIR}/_json_stub/CMakeLists.txt"
  "# nlohmann_json provided by system find_package; no targets created here.\n"
)
set(FETCHCONTENT_SOURCE_DIR_JSON
  "${CMAKE_BINARY_DIR}/_json_stub" CACHE PATH "" FORCE
)

FetchContent_Declare(minja
  GIT_REPOSITORY https://github.com/ochafik/minja.git
  GIT_TAG        143465a
)
set(MINJA_TEST_ENABLED OFF CACHE BOOL "" FORCE)
set(MINJA_EXAMPLE_ENABLED OFF CACHE BOOL "" FORCE)
set(MINJA_FUZZTEST_ENABLED OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(minja)

if(NOT TARGET minja::minja)
  add_library(minja::minja ALIAS minja)
endif()

install(TARGETS minja
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY ${minja_SOURCE_DIR}/include/
  DESTINATION include
)

generate_parameter_library(
  semantic_scene_forests_generation_parameters
  src/semantic_scene_forests_generation_parameters.yaml
)

add_library(
  semantic_scene_forests_generation_component SHARED
  src/semantic_scene_forests_generation.cpp
)
add_library(
  semantic_scene_forests_generation::semantic_scene_forests_generation_component
  ALIAS semantic_scene_forests_generation_component
)

target_compile_features(
  semantic_scene_forests_generation_component PUBLIC cxx_std_23
)
target_compile_definitions(
  semantic_scene_forests_generation_component
  PRIVATE SEMANTIC_SCENE_FORESTS_GENERATION_BUILDING_DLL
)

target_include_directories(semantic_scene_forests_generation_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(semantic_scene_forests_generation_component
  PUBLIC
    oai_api::oai_api
    minja::minja
    semantic_scene_forests::semantic_scene_forests
    ${semantic_scene_forests_msgs_TARGETS}
    ${semantic_scene_forests_object_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
    pluginlib::pluginlib
    cv_bridge::cv_bridge
    ament_index_cpp::ament_index_cpp
    nlohmann_json::nlohmann_json
    diagnostic_updater::diagnostic_updater
    semantic_scene_forests_utils::semantic_scene_forests_utils
    semantic_scene_forests_generation_parameters
)

rclcpp_components_register_node(semantic_scene_forests_generation_component
  PLUGIN "semantic_scene_forests_generation::SceneGraphGenerationNode"
  EXECUTABLE semantic_scene_forests_generation_node
)

add_library(
  semantic_scene_forests_generation_tasks SHARED
  src/tasks/color_task.cpp
  src/tasks/material_task.cpp
  src/tasks/manipulation_task.cpp
  src/tasks/category_task.cpp
  src/tasks/affordance_task.cpp
  src/tasks/task_plugins.cpp
)

target_compile_features(
  semantic_scene_forests_generation_tasks PUBLIC cxx_std_23
)

target_include_directories(semantic_scene_forests_generation_tasks
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks
)

target_link_libraries(semantic_scene_forests_generation_tasks
  PUBLIC
    semantic_scene_forests_generation_component
    minja::minja
    nlohmann_json::nlohmann_json
)

pluginlib_export_plugin_description_file(
  semantic_scene_forests_generation
  semantic_scene_forests_generation_plugins.xml
)

install(TARGETS
  semantic_scene_forests_generation_component
  semantic_scene_forests_generation_tasks
  semantic_scene_forests_generation_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES semantic_scene_forests_generation_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_components
  pluginlib
  semantic_scene_forests_msgs
  semantic_scene_forests_object_msgs
  semantic_scene_forests
  oai_api
  cv_bridge
  ament_index_cpp
  nlohmann_json
  fmt
  OpenCV
  minja
)

ament_package()
