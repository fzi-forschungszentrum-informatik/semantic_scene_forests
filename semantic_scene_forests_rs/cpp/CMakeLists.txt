cmake_minimum_required(VERSION 3.20)
project(ssf_rs_adapter LANGUAGES CXX)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(Boost REQUIRED)
find_package(fmt REQUIRED)
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(semantic_scene_forests REQUIRED)

add_library(ssf_rs_adapter STATIC bridge.cpp)
target_compile_features(ssf_rs_adapter PRIVATE cxx_std_23)
set_target_properties(ssf_rs_adapter
  PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ssf_rs_adapter
  PRIVATE ../include "${CXXBRIDGE_INCLUDE_DIR}")
target_link_libraries(ssf_rs_adapter
  PRIVATE semantic_scene_forests::semantic_scene_forests)

file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/cargo-links.txt" CONTENT
  "cargo:rustc-link-search=native=
  $<TARGET_FILE_DIR:semantic_scene_forests::semantic_scene_forests>\n
  cargo:rustc-link-lib=dylib=semantic_scene_forests\n")
install(TARGETS ssf_rs_adapter ARCHIVE DESTINATION lib)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cargo-links.txt" DESTINATION .)
