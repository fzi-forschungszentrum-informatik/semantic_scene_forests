# Copyright (c) 2026 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(semantic_scene_forests_object_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_updater REQUIRED)
# The core package does not export its own dependencies; find them first.
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(semantic_scene_forests REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(libzip REQUIRED)

# Header-only worker interface plus the shared dependencies, linked by the tracker and generation.
add_library(worker_interface INTERFACE)
target_include_directories(worker_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(worker_interface INTERFACE cxx_std_20)
target_link_libraries(worker_interface
  INTERFACE
    ${semantic_scene_forests_object_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    rclcpp::rclcpp
    diagnostic_updater::diagnostic_updater
    semantic_scene_forests::semantic_scene_forests
)

generate_parameter_library(scene_graph_manager_parameters
  src/scene_graph_manager_parameters.yaml
)

add_library(scene_graph_manager_component SHARED
  src/scene_graph_manager_node.cpp
  src/scene_graph_bundle.cpp
)
target_compile_features(scene_graph_manager_component PUBLIC cxx_std_20)
target_link_libraries(scene_graph_manager_component
  PUBLIC
    worker_interface
    cv_bridge::cv_bridge
    pluginlib::pluginlib
    rclcpp_components::component
    scene_graph_manager_parameters
  PRIVATE
    fmt::fmt
    nlohmann_json::nlohmann_json
    libzip::zip
)
rclcpp_components_register_node(scene_graph_manager_component
  PLUGIN "semantic_scene_forests_manager::SceneGraphManagerNode"
  EXECUTABLE scene_graph_manager_node
)

# Worker plugins that live in this package.
add_library(manager_workers SHARED
  src/pruning_worker.cpp
)
target_compile_features(manager_workers PUBLIC cxx_std_20)
target_link_libraries(manager_workers PUBLIC worker_interface pluginlib::pluginlib)
pluginlib_export_plugin_description_file(semantic_scene_forests_manager
  semantic_scene_forests_manager_workers.xml
)

install(TARGETS
  worker_interface
  scene_graph_manager_component
  scene_graph_manager_parameters
  manager_workers
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_scene_graph_manager_node test/test_scene_graph_manager_node.cpp
    TIMEOUT 120)
  target_include_directories(test_scene_graph_manager_node PRIVATE src)
  target_link_libraries(test_scene_graph_manager_node scene_graph_manager_component)

  ament_add_gtest(test_scene_graph_bundle test/test_scene_graph_bundle.cpp)
  target_include_directories(test_scene_graph_bundle PRIVATE src)
  target_link_libraries(test_scene_graph_bundle
    scene_graph_manager_component nlohmann_json::nlohmann_json libzip::zip)
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  diagnostic_updater
  rclcpp_components
  pluginlib
  Boost
  fmt
  semantic_scene_forests_msgs
  semantic_scene_forests
  semantic_scene_forests_object_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
)

ament_package()
