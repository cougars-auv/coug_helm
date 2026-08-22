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

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "coug_helm/bt_nodes/ros_bt_node.hpp"

namespace coug_helm::bt_nodes {

class EmergencySurface : public RosBtNode<BT::SyncActionNode> {
 public:
  EmergencySurface(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  // --- Overrides ---
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    RCLCPP_WARN(node_->get_logger(), "EmergencySurface: surfacing (not implemented).");
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
