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

class LoadCommand : public RosBtNode<BT::SyncActionNode> {
 public:
  LoadCommand(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<int>("pending_command"),
        BT::OutputPort<int>("active_command"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto pending = getPortOrBlackboard<int>("pending_command");
    if (pending != last_) {
      RCLCPP_INFO(node_->get_logger(), "LoadCommand: switching from %s to %s.",
                  utils::toString(static_cast<utils::AssistCommand>(last_)).c_str(),
                  utils::toString(static_cast<utils::AssistCommand>(pending)).c_str());
      last_ = pending;
    }
    setOutput("active_command", pending);
    return BT::NodeStatus::SUCCESS;
  }

 private:
  int last_{static_cast<int>(utils::AssistCommand::kStay)};
};

}  // namespace coug_helm::bt_nodes
