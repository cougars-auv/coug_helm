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
 * @file check_mission_complete.hpp
 * @brief BT condition node that checks whether the mission is finished.
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
 * @class CheckMissionComplete
 * @brief Returns SUCCESS if waypoint_index has advanced past the end of the waypoint list.
 */
class CheckMissionComplete : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor for CheckMissionComplete.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  CheckMissionComplete(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Point>>("waypoints"),
        BT::InputPort<size_t>("waypoint_index"),
    };
  }

  /**
   * @brief Checks whether the waypoint index has passed the end of the list.
   * @return SUCCESS if the mission is complete, FAILURE otherwise.
   */
  BT::NodeStatus tick() override {
    auto wps = getInput<std::vector<geometry_msgs::msg::Point>>("waypoints").value();
    auto idx = getInput<size_t>("waypoint_index").value();
    return (idx >= wps.size()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace coug_helm::bt_nodes
