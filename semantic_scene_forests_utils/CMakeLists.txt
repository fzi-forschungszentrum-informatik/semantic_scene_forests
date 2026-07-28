cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_library(semantic_scene_forests_utils INTERFACE)
add_library(semantic_scene_forests_utils::semantic_scene_forests_utils ALIAS
  semantic_scene_forests_utils)
target_include_directories(semantic_scene_forests_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(semantic_scene_forests_utils INTERFACE cxx_std_20)
target_link_libraries(semantic_scene_forests_utils INTERFACE
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS semantic_scene_forests_utils
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  diagnostic_msgs
  diagnostic_updater
)

ament_package()
