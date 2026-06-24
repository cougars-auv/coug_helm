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
 *        active_capture_radius and active_slip_radius with the home-specific values, causing
 *        the vehicle to navigate back to its launch position and surface there.
 */
class SetHomeWaypoint : public BT::SyncActionNode {
 public:
  SetHomeWaypoint(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Point>>("mission_waypoints"),
        BT::InputPort<double>("default_speed"),
        BT::InputPort<std::vector<double>>("home_capture_radius"),
        BT::InputPort<std::vector<double>>("home_slip_radius"),
        BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("active_waypoints"),
        BT::OutputPort<std::vector<double>>("active_waypoint_speeds"),
        BT::OutputPort<std::vector<double>>("active_capture_radius"),
        BT::OutputPort<std::vector<double>>("active_waypoint_capture_radii"),
        BT::OutputPort<std::vector<double>>("active_slip_radius"),
        BT::OutputPort<std::vector<double>>("active_waypoint_slip_radii"),
    };
  }

  /**
   * @brief Overwrites active_waypoints with mission_waypoints[0] at depth 0 at the default speed,
   * and sets home radii.
   * @return SUCCESS if a mission exists, FAILURE if none has been published.
   */
  BT::NodeStatus tick() override {
    auto mission = getInput<std::vector<geometry_msgs::msg::Point>>("mission_waypoints").value();
    if (mission.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::Point home = mission[0];
    home.z = 0.0;
    setOutput("active_waypoints", std::vector<geometry_msgs::msg::Point>{home});
    setOutput("active_waypoint_speeds",
              std::vector<double>{getInput<double>("default_speed").value()});
    setOutput("active_capture_radius",
              getInput<std::vector<double>>("home_capture_radius").value());
    setOutput("active_waypoint_capture_radii", std::vector<double>{});
    setOutput("active_slip_radius", getInput<std::vector<double>>("home_slip_radius").value());
    setOutput("active_waypoint_slip_radii", std::vector<double>{});
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
