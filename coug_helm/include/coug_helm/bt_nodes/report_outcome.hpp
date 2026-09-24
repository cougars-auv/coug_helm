// Copyright 2026 BYU FROST Lab
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
#include "coug_helm/utils/behavior_enums.hpp"

namespace coug_helm::bt_nodes {

class ReportOutcome : public RosBtNode<BT::DecoratorNode> {
 public:
  ReportOutcome(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::DecoratorNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList { return {BT::InputPort<int>("active_behavior")}; }

  auto tick() -> BT::NodeStatus override {
    setStatus(BT::NodeStatus::RUNNING);
    const BT::NodeStatus child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::RUNNING) {
      return child_status;
    }
    resetChild();

    const auto behavior =
        utils::toString(static_cast<utils::Behavior>(getInput<int>("active_behavior").value()));
    if (child_status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "ReportOutcome: %s complete.", behavior.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "ReportOutcome: %s failed.", behavior.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace coug_helm::bt_nodes
