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

class AbortOnTeleop : public RosBtNode<BT::SyncActionNode> {
 public:
  AbortOnTeleop(const std::string& name, const BT::NodeConfig& config)
      : RosBtNode<BT::SyncActionNode>(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<double>("last_teleop_time"),
        BT::BidirectionalPort<int>("pending_behavior"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const auto last_teleop_time = getPortOrBlackboard<double>("last_teleop_time");
    if (last_teleop_time <= last_seen_teleop_time_) {
      return BT::NodeStatus::SUCCESS;
    }
    last_seen_teleop_time_ = last_teleop_time;

    const auto pending = static_cast<utils::Behavior>(getInput<int>("pending_behavior").value());
    if (pending != utils::Behavior::kStop && !utils::isEmergency(pending)) {
      RCLCPP_WARN(node_->get_logger(), "AbortOnTeleop: aborting %s.",
                  utils::toString(pending).c_str());
      setOutput("pending_behavior", static_cast<int>(utils::Behavior::kStop));
    }
    return BT::NodeStatus::SUCCESS;
  }

 private:
  double last_seen_teleop_time_{-1.0};
};

}  // namespace coug_helm::bt_nodes
