// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file waypoint.hpp
 * @brief Waypoint structure grouping target parameters.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <geometry_msgs/msg/point.hpp>

namespace coug_helm::utils {

/**
 * @struct Waypoint
 * @brief Container representing a vehicle waypoint with associated target speed and radii.
 */
struct Waypoint {
  geometry_msgs::msg::Point position;
  double speed{0.0};
  double capture_radius_horizontal{0.0};
  double capture_radius_vertical{0.0};
  double slip_radius_horizontal{0.0};
  double slip_radius_vertical{0.0};
};

}  // namespace coug_helm::utils
