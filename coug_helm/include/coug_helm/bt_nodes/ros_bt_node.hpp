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
 * @file ros_bt_node.hpp
 * @brief Base mixin that gives a BT leaf access to the shared ROS node.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace coug_helm::bt_nodes {

/**
 * @class RosBtNode
 * @brief Gives a BT leaf the shared rclcpp::Node from the blackboard.
 * @tparam BTBase The BehaviorTree.CPP base class (ConditionNode, SyncActionNode, ...).
 */
template <class BTBase>
class RosBtNode : public BTBase {
 public:
  /**
   * @brief Constructs the BT node and caches the shared rclcpp::Node from the blackboard.
   * @param name The BT node instance name.
   * @param config The BT node configuration (provides the blackboard).
   */
  RosBtNode(const std::string& name, const BT::NodeConfig& config)
      : BTBase(name, config), node_(config.blackboard->template get<rclcpp::Node*>("node")) {}

 protected:
  rclcpp::Node* node_;
};

}  // namespace coug_helm::bt_nodes
