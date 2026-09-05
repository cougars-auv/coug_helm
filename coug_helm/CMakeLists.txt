# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(coug_helm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  bt_helm_parameters
  "params/bt_helm_params.yaml"
)

ament_auto_add_library(bt_helm_component SHARED NO_TARGET_LINK_LIBRARIES
  src/bt_helm.cpp
)

target_compile_features(bt_helm_component PUBLIC cxx_std_17)

target_link_libraries(bt_helm_component
  bt_helm_parameters
)

rclcpp_components_register_nodes(bt_helm_component
  "coug_helm::BtHelmNode"
)

ament_auto_add_executable(bt_helm NO_TARGET_LINK_LIBRARIES
  src/bt_helm_node.cpp
)
target_link_libraries(bt_helm
  bt_helm_component
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  trees
  USE_SCOPED_HEADER_INSTALL_DIR
)
