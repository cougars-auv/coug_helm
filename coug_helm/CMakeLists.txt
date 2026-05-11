cmake_minimum_required(VERSION 3.22)
project(coug_helm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED IMPORTED_TARGET geographiclib)

find_package(generate_parameter_library REQUIRED)

# --- Parameters ---
generate_parameter_library(
  bt_helm_parameters
  "params/bt_helm_params.yaml"
)

# --- Component Library ---
ament_auto_add_library(bt_helm_component SHARED
  src/bt_helm.cpp
)

target_include_directories(bt_helm_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(bt_helm_component
  bt_helm_parameters
  PkgConfig::GeographicLib
)

rclcpp_components_register_nodes(bt_helm_component
  "coug_helm::BTHelmNode"
)

# --- Executables ---
ament_auto_add_executable(bt_helm src/bt_helm_node.cpp)
target_link_libraries(bt_helm
  bt_helm_component
)

# --- Testing ---
if(BUILD_TESTING)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  trees
  USE_SCOPED_HEADER_INSTALL_DIR
)
