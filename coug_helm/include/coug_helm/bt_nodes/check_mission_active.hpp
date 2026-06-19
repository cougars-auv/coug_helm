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
 * @file check_mission_active.hpp
 * @brief BT condition node that checks whether a mission is actively running.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>

namespace coug_helm::bt_nodes {

/**
 * @class CheckMissionActive
 * @brief Returns SUCCESS if a non-empty waypoint list is loaded and mission_active is true.
 */
class CheckMissionActive : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor for CheckMissionActive.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  CheckMissionActive(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Point>>("waypoints"),
        BT::InputPort<bool>("mission_active"),
    };
  }

  /**
   * @brief Checks whether waypoints are loaded and the mission is active.
   * @return SUCCESS if mission is active and waypoints are available, FAILURE otherwise.
   */
  BT::NodeStatus tick() override {
    auto wps = getInput<std::vector<geometry_msgs::msg::Point>>("waypoints").value();
    bool active = getInput<bool>("mission_active").value();
    return (!wps.empty() && active) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace coug_helm::bt_nodes
