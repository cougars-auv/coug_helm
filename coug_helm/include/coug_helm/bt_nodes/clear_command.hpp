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
 * @file clear_command.hpp
 * @brief BT action node that clears the pending command.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <string>

namespace coug_helm::bt_nodes {

/**
 * @class ClearCommand
 * @brief Clears the pending_command entry on the blackboard so the command is not re-dispatched.
 */
class ClearCommand : public BT::SyncActionNode {
 public:
  /**
   * @brief Constructor for ClearCommand.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  ClearCommand(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {BT::OutputPort<std::string>("pending_command")}; }

  /**
   * @brief Sets pending_command to an empty string.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    setOutput("pending_command", std::string{""});
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
