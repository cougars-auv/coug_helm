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
 * @file check_has_mission.hpp
 * @brief BT condition node that checks whether a mission has been received.
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
 * @class CheckHasMission
 * @brief Returns SUCCESS if a non-empty waypoint list is loaded.
 */
class CheckHasMission : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor for CheckHasMission.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  CheckHasMission(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Checks whether a non-empty waypoint list is available.
   * @return SUCCESS if waypoints are loaded, FAILURE otherwise.
   */
  BT::NodeStatus tick() override {
    auto wps = config().blackboard->get<std::vector<geometry_msgs::msg::Point>>("waypoints");
    return wps.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
