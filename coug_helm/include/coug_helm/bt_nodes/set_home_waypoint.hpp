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
 * @file set_home_waypoint.hpp
 * @brief BT action node that replaces the waypoint list with the home waypoint.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace coug_helm::bt_nodes {

/**
 * @class SetHomeWaypoint
 * @brief Replaces the waypoint list with the first mission waypoint at depth 0 and overrides
 *        capture_radius and slip_radius with the home-specific values, causing the vehicle
 *        to navigate back to its launch position and surface there.
 */
class SetHomeWaypoint : public BT::SyncActionNode {
 public:
  SetHomeWaypoint(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Overwrites waypoints with mission_waypoints[0] at depth 0, and sets home radii.
   * @return SUCCESS if a mission exists, FAILURE if none has been published.
   */
  BT::NodeStatus tick() override {
    auto mission =
        config().blackboard->get<std::vector<geometry_msgs::msg::Point>>("mission_waypoints");
    if (mission.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::Point home = mission[0];
    home.z = 0.0;
    config().blackboard->set("waypoints", std::vector<geometry_msgs::msg::Point>{home});
    config().blackboard->set("capture_radius",
                             config().blackboard->get<std::vector<double>>("home_capture_radius"));
    config().blackboard->set("slip_radius",
                             config().blackboard->get<std::vector<double>>("home_slip_radius"));
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
