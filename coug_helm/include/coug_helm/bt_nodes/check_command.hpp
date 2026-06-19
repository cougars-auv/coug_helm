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
 * @file check_command.hpp
 * @brief BT condition node that checks for a specific pending command.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <string>

namespace coug_helm::bt_nodes {

/**
 * @class CheckCommand
 * @brief Returns SUCCESS if pending_command on the blackboard matches the "command" port value.
 */
class CheckCommand : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor for CheckCommand.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  CheckCommand(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("command"),
        BT::InputPort<std::string>("pending_command"),
    };
  }

  /**
   * @brief Compares pending_command against the "command" port value.
   * @return SUCCESS if they match, FAILURE otherwise.
   */
  BT::NodeStatus tick() override {
    auto expected = getInput<std::string>("command").value();
    auto pending = getInput<std::string>("pending_command").value();
    return (pending == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace coug_helm::bt_nodes
