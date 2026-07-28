cmake_minimum_required(VERSION 3.20)
project(semantic_scene_forests)

if (POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" VERSION_XML)

# Use a regular expression to extract the version number
# (assumes a pattern like <version>1.2.3</version>)
string(
  REGEX MATCH
  "<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>"
  VERSION_MATCH
  "${VERSION_XML}"
)

if (NOT VERSION_MATCH)
  message(FATAL_ERROR "Could not extract version from version.xml")
endif ()

# Extract major, minor, and patch numbers from the regex capture groups
set(PROJECT_VERSION_MAJOR ${CMAKE_MATCH_1})
set(PROJECT_VERSION_MINOR ${CMAKE_MATCH_2})
set(PROJECT_VERSION_PATCH ${CMAKE_MATCH_3})
set(PROJECT_VERSION
  "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}"
)

configure_file(
        "resources/version.hpp.in"
        "${CMAKE_CURRENT_BINARY_DIR}/include/semantic_scene_forests/version.hpp"
        @ONLY
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(Boost REQUIRED)
find_package(fmt REQUIRED)

add_library(semantic_scene_forests SHARED src/scene_graph.cpp)
add_library(
  semantic_scene_forests::semantic_scene_forests ALIAS semantic_scene_forests)

target_compile_features(semantic_scene_forests PUBLIC c_std_11 cxx_std_23)
# Require C99 and C++17
target_include_directories(semantic_scene_forests PUBLIC
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
        semantic_scene_forests PUBLIC
        ${semantic_scene_forests_msgs_TARGETS}
        ${vision_msgs_TARGETS}
        Boost::boost
        fmt::fmt
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(
  semantic_scene_forests PRIVATE "SEMANTIC_SCENE_FORESTS_BUILDING_LIBRARY")

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
install(
        DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
        DESTINATION include/${PROJECT_NAME}
)
install(
        TARGETS semantic_scene_forests
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_scene_graph test/test_scene_graph.cpp)
  target_link_libraries(
    test_scene_graph semantic_scene_forests::semantic_scene_forests)
endif ()

ament_export_include_directories(
        "include/${PROJECT_NAME}"
)
ament_export_libraries(
        semantic_scene_forests
)
ament_export_targets(
        export_${PROJECT_NAME}
)

ament_package()
