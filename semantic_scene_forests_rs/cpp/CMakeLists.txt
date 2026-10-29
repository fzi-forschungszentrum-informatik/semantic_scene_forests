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

file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/cargo-links.txt" CONTENT "cargo:rustc-link-search=native=$<TARGET_FILE_DIR:semantic_scene_forests::semantic_scene_forests>\ncargo:rustc-link-lib=dylib=semantic_scene_forests\n")
install(TARGETS ssf_rs_adapter ARCHIVE DESTINATION lib)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cargo-links.txt" DESTINATION .)
