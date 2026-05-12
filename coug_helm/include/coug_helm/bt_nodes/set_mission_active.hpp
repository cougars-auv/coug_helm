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
 * @file set_mission_active.hpp
 * @brief BT action node that sets the mission_active flag.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace coug_helm::bt_nodes {

/**
 * @class SetMissionActive
 * @brief Enables or disables mission execution by writing the "value" port to mission_active.
 */
class SetMissionActive : public BT::SyncActionNode {
 public:
  /**
   * @brief Constructor for SetMissionActive.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  SetMissionActive(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("value", "true to enable, false to disable")};
  }

  /**
   * @brief Writes the "value" port to mission_active on the blackboard.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    auto value = getInput<bool>("value").value();
    config().blackboard->set("mission_active", value);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
