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
 * @file compute_surface_waypoint.hpp
 * @brief BT action node that computes a single surface waypoint.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"
#include "coug_helm/waypoint.hpp"

namespace coug_helm::bt_nodes {

/**
 * @class ComputeSurfaceWaypoint
 * @brief Generates a single depth-0 surface waypoint at the vehicle's current position.
 */
class ComputeSurfaceWaypoint : public RosBtNode<BT::SyncActionNode> {
 public:
  ComputeSurfaceWaypoint(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  // --- Overrides ---
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("current_x"),
        BT::InputPort<double>("current_y"),
        BT::InputPort<double>("default_speed"),
        BT::InputPort<std::vector<double>>("surface_capture_radius"),
        BT::InputPort<std::vector<double>>("surface_slip_radius"),
        BT::OutputPort<std::vector<Waypoint>>("surface_waypoint"),
    };
  }

  /**
   * @brief Stages a surface waypoint at (current_x, current_y, z=0).
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    Waypoint wp;
    wp.position.x = getInput<double>("current_x").value();
    wp.position.y = getInput<double>("current_y").value();
    wp.position.z = 0.0;
    wp.speed = getInput<double>("default_speed").value();

    auto cap = getInput<std::vector<double>>("surface_capture_radius").value();
    wp.capture_radius_horizontal = cap[0];
    wp.capture_radius_vertical = cap[1];

    auto slip = getInput<std::vector<double>>("surface_slip_radius").value();
    wp.slip_radius_horizontal = slip[0];
    wp.slip_radius_vertical = slip[1];

    RCLCPP_INFO(node_->get_logger(),
                "ComputeSurfaceWaypoint: surface set to (%.1f, %.1f), depth 0.", wp.position.x,
                wp.position.y);
    setOutput("surface_waypoint", std::vector<Waypoint>{wp});
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
