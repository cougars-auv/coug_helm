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
 * @file load_behavior.hpp
 * @brief BT action node that loads the pending behavior into the active behavior.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"
#include "coug_helm/utils/behavior_enums.hpp"

namespace coug_helm::bt_nodes {

/**
 * @class LoadBehavior
 * @brief Copies the pending behavior into the active behavior for the tree to dispatch on.
 */
class LoadBehavior : public RosBtNode<BT::SyncActionNode> {
 public:
  LoadBehavior(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  // --- Overrides ---
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("pending_behavior"),
        BT::OutputPort<int>("active_behavior"),
    };
  }

  /**
   * @brief Copies the pending behavior into the active behavior, logging on change.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    int pending = getInput<int>("pending_behavior").value();
    if (pending != last_behavior_) {
      RCLCPP_INFO(node_->get_logger(), "LoadBehavior: %s -> %s",
                  utils::toString(static_cast<utils::Behavior>(last_behavior_)).c_str(),
                  utils::toString(static_cast<utils::Behavior>(pending)).c_str());
      last_behavior_ = pending;
    }
    setOutput("active_behavior", pending);
    return BT::NodeStatus::SUCCESS;
  }

 private:
  int last_behavior_{static_cast<int>(utils::Behavior::STOP)};
};

}  // namespace coug_helm::bt_nodes
