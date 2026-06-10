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
 * @file set_mission_waypoints.hpp
 * @brief BT action node that loads the published mission back into the active waypoint list.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace coug_helm::bt_nodes {

/**
 * @class SetMissionWaypoints
 * @brief Copies mission_waypoints into waypoints and resets the waypoint speeds,
 *        capture_radius, and slip_radius to the mission values, undoing any overrides
 *        left by surface or home commands.
 */
class SetMissionWaypoints : public BT::SyncActionNode {
 public:
  SetMissionWaypoints(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Sets waypoints and waypoint_speeds from the stored mission values.
   * @return SUCCESS if a mission exists, FAILURE if none has been published.
   */
  BT::NodeStatus tick() override {
    auto mission =
        config().blackboard->get<std::vector<geometry_msgs::msg::Point>>("mission_waypoints");
    if (mission.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set("waypoints", mission);
    config().blackboard->set("waypoint_speeds", config().blackboard->get<std::vector<double>>(
                                                    "mission_waypoint_speeds"));
    config().blackboard->set(
        "capture_radius", config().blackboard->get<std::vector<double>>("mission_capture_radius"));
    config().blackboard->set("slip_radius",
                             config().blackboard->get<std::vector<double>>("mission_slip_radius"));
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
