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
 * @file set_surface_waypoint.hpp
 * @brief BT action node that replaces the waypoint list with a single surface waypoint.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace coug_helm::bt_nodes {

/**
 * @class SetSurfaceWaypoint
 * @brief Replaces the waypoint list with a single depth-0 waypoint at the vehicle's current
 *        ENU (x, y) position, causing the vehicle to surface in place.
 *        Also overrides capture_radius and slip_radius with the surface-specific values.
 */
class SetSurfaceWaypoint : public BT::SyncActionNode {
 public:
  /**
   * @brief Constructor for SetSurfaceWaypoint.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  SetSurfaceWaypoint(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Overwrites waypoints with one entry at (current_x, current_y, z=0) at the default
   * speed, and sets surface radii.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    geometry_msgs::msg::Point wp;
    wp.x = config().blackboard->get<double>("current_x");
    wp.y = config().blackboard->get<double>("current_y");
    wp.z = 0.0;
    config().blackboard->set("waypoints", std::vector<geometry_msgs::msg::Point>{wp});
    config().blackboard->set(
        "waypoint_speeds", std::vector<double>{config().blackboard->get<double>("default_speed")});
    config().blackboard->set(
        "capture_radius", config().blackboard->get<std::vector<double>>("surface_capture_radius"));
    config().blackboard->set("slip_radius",
                             config().blackboard->get<std::vector<double>>("surface_slip_radius"));
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
