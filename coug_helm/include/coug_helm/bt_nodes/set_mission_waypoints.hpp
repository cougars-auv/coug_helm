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
 * @brief Copies mission_waypoints into active_waypoints and resets active_waypoint_speeds,
 *        active_capture_radius, active_slip_radius, and the active per-waypoint radii to the
 *        mission values, undoing any overrides left by surface or home commands.
 */
class SetMissionWaypoints : public BT::SyncActionNode {
 public:
  SetMissionWaypoints(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Point>>("mission_waypoints"),
        BT::InputPort<std::vector<double>>("mission_waypoint_speeds"),
        BT::InputPort<std::vector<double>>("default_capture_radius"),
        BT::InputPort<std::vector<double>>("mission_waypoint_capture_radii"),
        BT::InputPort<std::vector<double>>("default_slip_radius"),
        BT::InputPort<std::vector<double>>("mission_waypoint_slip_radii"),
        BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("active_waypoints"),
        BT::OutputPort<std::vector<double>>("active_waypoint_speeds"),
        BT::OutputPort<std::vector<double>>("active_capture_radius"),
        BT::OutputPort<std::vector<double>>("active_waypoint_capture_radii"),
        BT::OutputPort<std::vector<double>>("active_slip_radius"),
        BT::OutputPort<std::vector<double>>("active_waypoint_slip_radii"),
    };
  }

  /**
   * @brief Sets active_waypoints and active_waypoint_speeds from the stored mission values.
   * @return SUCCESS if a mission exists, FAILURE if none has been published.
   */
  BT::NodeStatus tick() override {
    auto mission = getInput<std::vector<geometry_msgs::msg::Point>>("mission_waypoints").value();
    if (mission.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("active_waypoints", mission);
    setOutput("active_waypoint_speeds",
              getInput<std::vector<double>>("mission_waypoint_speeds").value());
    setOutput("active_capture_radius",
              getInput<std::vector<double>>("default_capture_radius").value());
    setOutput("active_waypoint_capture_radii",
              getInput<std::vector<double>>("mission_waypoint_capture_radii").value());
    setOutput("active_slip_radius", getInput<std::vector<double>>("default_slip_radius").value());
    setOutput("active_waypoint_slip_radii",
              getInput<std::vector<double>>("mission_waypoint_slip_radii").value());
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
