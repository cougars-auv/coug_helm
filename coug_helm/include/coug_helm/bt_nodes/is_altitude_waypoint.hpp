// Copyright 2026 BYU FROST Lab
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

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <coug_interfaces/msg/way_point.hpp>
#include <cstddef>
#include <string>
#include <vector>

namespace coug_helm::bt_nodes {

class IsAltitudeWaypoint : public BT::ConditionNode {
 public:
  IsAltitudeWaypoint(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints"),
        BT::InputPort<size_t>("current_waypoint"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto waypoints =
        getInput<std::vector<coug_interfaces::msg::WayPoint>>("active_waypoints").value();
    const size_t waypoint_idx = getInput<size_t>("current_waypoint").value();

    if (waypoint_idx >= waypoints.size()) {
      return BT::NodeStatus::FAILURE;
    }
    return (waypoints[waypoint_idx].mode == coug_interfaces::msg::WayPoint::ALTITUDE)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
  }
};

}  // namespace coug_helm::bt_nodes
