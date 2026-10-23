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
project(semantic_scene_forests_object_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(semantic_scene_forests_manager REQUIRED)
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
find_package(oai_api REQUIRED)
find_package(nlohmann_json REQUIRED)

generate_parameter_library(object_tracker_parameters
  src/object_tracker_parameters.yaml
)

# Tracker worker plugin (hosted by the scene_graph_manager), also linked by the tests.
add_library(tracker_worker SHARED
  src/tracker_worker.cpp
  src/arbiter_worker.cpp
  src/camera_tracker.cpp
  src/global_tracker.cpp
  src/image_rotation.cpp
  src/tracked_object.cpp
  src/tracker_geometry.cpp
)
target_include_directories(tracker_worker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(tracker_worker PUBLIC cxx_std_23)
target_link_libraries(tracker_worker PUBLIC
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
  pluginlib::pluginlib
  semantic_scene_forests_manager::world_db
  diagnostic_updater::diagnostic_updater
  semantic_scene_forests_utils::semantic_scene_forests_utils
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  object_tracker_parameters
  oai_api::oai_api
  nlohmann_json::nlohmann_json
)
pluginlib_export_plugin_description_file(semantic_scene_forests_manager
  semantic_scene_forests_object_tracker_workers.xml
)

install(TARGETS
  tracker_worker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bbox_transform test/test_bbox_transform.cpp)
  target_link_libraries(test_bbox_transform tracker_worker)
  ament_add_gtest(test_tracked_object test/test_tracked_object.cpp)
  target_link_libraries(test_tracked_object tracker_worker)
  target_include_directories(test_tracked_object PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  foreach(unit_test test_image_rotation test_camera_tracker test_global_tracker test_arbiter_config)
    ament_add_gtest(${unit_test} test/${unit_test}.cpp)
    target_link_libraries(${unit_test} tracker_worker)
    target_include_directories(${unit_test} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  endforeach()
  target_include_directories(test_bbox_transform PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

  ament_add_gtest(test_multi_camera_tracker test/test_multi_camera_tracker.cpp)
  target_link_libraries(test_multi_camera_tracker tracker_worker)
endif()

ament_package()
