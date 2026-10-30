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
project(semantic_scene_forests_query)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Core ROS2 / ament ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Messages / services / actions ────────────────────────────────────────────
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(semantic_scene_forests_object_msgs REQUIRED)
find_package(semantic_scene_forests_planning_msgs REQUIRED)
find_package(semantic_scene_forests REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# ── Third-party ───────────────────────────────────────────────────────────────
find_package(oai_api REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(semantic_scene_forests_utils REQUIRED)

set(JSON_Install OFF CACHE BOOL "" FORCE)
include(FetchContent)
FetchContent_Declare(json
  URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.zip
  FIND_PACKAGE_ARGS
  NAMES nlohmann_json
)
FetchContent_Declare(minja
  GIT_REPOSITORY https://github.com/ochafik/minja.git
  GIT_TAG        143465a  # pinned: "Add support for Mistral 3 series templates"
)
set(MINJA_TEST_ENABLED     OFF CACHE BOOL "" FORCE)
set(MINJA_EXAMPLE_ENABLED  OFF CACHE BOOL "" FORCE)
set(MINJA_FUZZTEST_ENABLED OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(minja)
if(NOT TARGET minja::minja)
  add_library(minja::minja ALIAS minja)
endif()

# ── Generated parameter struct ────────────────────────────────────────────────
generate_parameter_library(semantic_scene_forests_query_parameters
  src/semantic_scene_forests_query_parameters.yaml
)

# ── Component library ─────────────────────────────────────────────────────────
add_library(semantic_scene_forests_query_component SHARED
  src/semantic_scene_forests_query_node.cpp
  src/graph_agent.cpp
)
add_library(semantic_scene_forests_query::semantic_scene_forests_query_component
  ALIAS semantic_scene_forests_query_component)

target_compile_features(
  semantic_scene_forests_query_component PUBLIC cxx_std_23)
target_compile_definitions(semantic_scene_forests_query_component
  PRIVATE SEMANTIC_SCENE_FORESTS_QUERY_BUILDING_DLL)

target_include_directories(semantic_scene_forests_query_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(semantic_scene_forests_query_component
  PUBLIC
    oai_api::oai_api
    minja::minja
    semantic_scene_forests::semantic_scene_forests
    ${semantic_scene_forests_msgs_TARGETS}
    ${semantic_scene_forests_object_msgs_TARGETS}
    ${semantic_scene_forests_planning_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
    cv_bridge::cv_bridge
    ament_index_cpp::ament_index_cpp
    nlohmann_json::nlohmann_json
    diagnostic_updater::diagnostic_updater
    semantic_scene_forests_utils::semantic_scene_forests_utils
    fmt::fmt
    semantic_scene_forests_query_parameters
)

# Register the component and autogenerate a standalone executable.
rclcpp_components_register_node(semantic_scene_forests_query_component
  PLUGIN "semantic_scene_forests_query::SceneGraphQueryNode"
  EXECUTABLE semantic_scene_forests_query_node
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS minja
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY ${minja_SOURCE_DIR}/include/
  DESTINATION include
)

install(TARGETS
  semantic_scene_forests_query_component
  semantic_scene_forests_query_parameters
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

ament_package()
