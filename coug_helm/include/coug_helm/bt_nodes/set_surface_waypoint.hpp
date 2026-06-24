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
 *        Also overrides active_capture_radius and active_slip_radius with the surface-specific
 *        values.
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

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("current_x"),
        BT::InputPort<double>("current_y"),
        BT::InputPort<double>("default_speed"),
        BT::InputPort<std::vector<double>>("surface_capture_radius"),
        BT::InputPort<std::vector<double>>("surface_slip_radius"),
        BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("active_waypoints"),
        BT::OutputPort<std::vector<double>>("active_waypoint_speeds"),
        BT::OutputPort<std::vector<double>>("active_capture_radius"),
        BT::OutputPort<std::vector<double>>("active_waypoint_capture_radii"),
        BT::OutputPort<std::vector<double>>("active_slip_radius"),
        BT::OutputPort<std::vector<double>>("active_waypoint_slip_radii"),
    };
  }

  /**
   * @brief Overwrites active_waypoints with one entry at (current_x, current_y, z=0) at the default
   * speed, and sets surface radii.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    geometry_msgs::msg::Point wp;
    wp.x = getInput<double>("current_x").value();
    wp.y = getInput<double>("current_y").value();
    wp.z = 0.0;
    setOutput("active_waypoints", std::vector<geometry_msgs::msg::Point>{wp});
    setOutput("active_waypoint_speeds",
              std::vector<double>{getInput<double>("default_speed").value()});
    setOutput("active_capture_radius",
              getInput<std::vector<double>>("surface_capture_radius").value());
    setOutput("active_waypoint_capture_radii", std::vector<double>{});
    setOutput("active_slip_radius", getInput<std::vector<double>>("surface_slip_radius").value());
    setOutput("active_waypoint_slip_radii", std::vector<double>{});
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
