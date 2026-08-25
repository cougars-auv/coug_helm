cmake_minimum_required(VERSION 3.22)
project(coug_helm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED IMPORTED_TARGET geographiclib)

# --- Parameters ---
generate_parameter_library(
  bt_helm_parameters
  "params/bt_helm_params.yaml"
)

# --- Component Library ---
ament_auto_add_library(bt_helm_component SHARED NO_TARGET_LINK_LIBRARIES
  src/bt_helm.cpp
)

target_compile_features(bt_helm_component PUBLIC cxx_std_17)

target_link_libraries(bt_helm_component
  bt_helm_parameters
  PkgConfig::GeographicLib
)

rclcpp_components_register_nodes(bt_helm_component
  "coug_helm::BtHelmNode"
)

# --- Executables ---
ament_auto_add_executable(bt_helm NO_TARGET_LINK_LIBRARIES
  src/bt_helm_node.cpp
)
target_link_libraries(bt_helm
  bt_helm_component
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  trees
  USE_SCOPED_HEADER_INSTALL_DIR
)
