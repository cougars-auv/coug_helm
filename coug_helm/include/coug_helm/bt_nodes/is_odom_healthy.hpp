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

#include <string>

namespace coug_helm::bt_nodes {

class IsOdomHealthy : public BT::ConditionNode {
 public:
  IsOdomHealthy(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<double>("last_odom_time"),
        BT::InputPort<double>("current_time"),
        BT::InputPort<double>("odom_timeout"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const double last_odom = getInput<double>("last_odom_time").value();
    const double current_time = getInput<double>("current_time").value();
    const double timeout = getInput<double>("odom_timeout").value();

    if (last_odom == 0.0) {
      return BT::NodeStatus::FAILURE;
    }
    return ((current_time - last_odom) < timeout) ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::FAILURE;
  }
};

}  // namespace coug_helm::bt_nodes
