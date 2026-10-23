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
project(semantic_scene_forests_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core ROS2 / ament
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
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
find_package(diagnostic_updater REQUIRED)

# Third-party
find_package(oai_api REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(semantic_scene_forests_manager REQUIRED)

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

# LLM client, template and task plugin loading shared by the workers, plus their goal and
# thread handling.
add_library(generation_common SHARED
  src/generation_common.cpp
)
target_compile_features(generation_common PUBLIC cxx_std_23)
target_include_directories(generation_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(generation_common
  PUBLIC
    oai_api::oai_api
    minja::minja
    semantic_scene_forests::semantic_scene_forests
    ${semantic_scene_forests_object_msgs_TARGETS}
    ${OpenCV_LIBS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    diagnostic_updater::diagnostic_updater
    ${semantic_scene_forests_msgs_TARGETS}
    pluginlib::pluginlib
    nlohmann_json::nlohmann_json
  PRIVATE
    ament_index_cpp::ament_index_cpp
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
    generation_common
    minja::minja
    nlohmann_json::nlohmann_json
)

pluginlib_export_plugin_description_file(
  semantic_scene_forests_generation
  semantic_scene_forests_generation_plugins.xml
)

generate_parameter_library(object_attribution_parameters
  src/object_attribution_parameters.yaml
)

# Worker plugin hosted by the scene_graph_manager.
add_library(object_attribution_worker SHARED
  src/object_attribution_worker.cpp
)
target_compile_features(object_attribution_worker PUBLIC cxx_std_23)
target_link_libraries(object_attribution_worker
  PUBLIC
    generation_common
    object_attribution_parameters
    semantic_scene_forests_manager::world_db
    ${semantic_scene_forests_msgs_TARGETS}
    rclcpp_action::rclcpp_action
    cv_bridge::cv_bridge
)
generate_parameter_library(concept_generation_parameters
  src/concept_generation_parameters.yaml
)

add_library(concept_generation_worker SHARED
  src/concept_generation_worker.cpp
)
target_compile_features(concept_generation_worker PUBLIC cxx_std_23)
target_link_libraries(concept_generation_worker
  PUBLIC
    generation_common
    concept_generation_parameters
    semantic_scene_forests_manager::world_db
    ${semantic_scene_forests_msgs_TARGETS}
    rclcpp_action::rclcpp_action
)

pluginlib_export_plugin_description_file(semantic_scene_forests_manager
  semantic_scene_forests_generation_workers.xml
)

install(TARGETS
  concept_generation_parameters
  concept_generation_worker
  generation_common
  object_attribution_parameters
  object_attribution_worker
  semantic_scene_forests_generation_tasks
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

install(FILES semantic_scene_forests_generation_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_object_attribution_worker test/test_object_attribution_worker.cpp
    TIMEOUT 60)
  target_include_directories(test_object_attribution_worker PRIVATE src)
  target_link_libraries(test_object_attribution_worker object_attribution_worker)

  ament_add_gtest(test_concept_generation_worker test/test_concept_generation_worker.cpp
    TIMEOUT 60)
  target_include_directories(test_concept_generation_worker PRIVATE src)
  target_link_libraries(test_concept_generation_worker concept_generation_worker)
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  pluginlib
  semantic_scene_forests_msgs
  semantic_scene_forests_object_msgs
  semantic_scene_forests
  semantic_scene_forests_manager
  oai_api
  cv_bridge
  ament_index_cpp
  nlohmann_json
  fmt
  OpenCV
  minja
)

ament_package()
