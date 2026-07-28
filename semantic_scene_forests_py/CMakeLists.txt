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
project(semantic_scene_forests_py)

if (POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)
find_package(semantic_scene_forests REQUIRED)
find_package(semantic_scene_forests_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_semantic_scene_forests src/bindings.cpp)
target_compile_features(_semantic_scene_forests PRIVATE cxx_std_23)
target_link_libraries(
  _semantic_scene_forests
  PRIVATE
    semantic_scene_forests::semantic_scene_forests
    rclcpp::rclcpp
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(
  TARGETS _semantic_scene_forests
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/semantic_scene_forests"
)
ament_python_install_package(
  semantic_scene_forests
  PACKAGE_DIR python/semantic_scene_forests
)

if (BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  file(
    COPY python/semantic_scene_forests
    DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/python"
  )
  set_target_properties(
    _semantic_scene_forests
    PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY
      "${CMAKE_CURRENT_BINARY_DIR}/python/semantic_scene_forests"
  )

  ament_add_pytest_test(
    test_scene_graph
    test/test_scene_graph.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/python"
  )
endif ()

ament_package()
