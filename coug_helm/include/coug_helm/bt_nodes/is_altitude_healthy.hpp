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

class IsAltitudeHealthy : public BT::ConditionNode {
 public:
  IsAltitudeHealthy(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        BT::InputPort<double>("last_altitude_time"),
        BT::InputPort<bool>("has_altitude"),
        BT::InputPort<double>("current_time"),
        BT::InputPort<double>("altitude_timeout"),
    };
  }

  auto tick() -> BT::NodeStatus override {
    const double last_altitude = getInput<double>("last_altitude_time").value();
    const bool has_altitude = getInput<bool>("has_altitude").value();
    const double current_time = getInput<double>("current_time").value();
    const double timeout = getInput<double>("altitude_timeout").value();

    if (!has_altitude) {
      return BT::NodeStatus::FAILURE;
    }
    return ((current_time - last_altitude) < timeout) ? BT::NodeStatus::SUCCESS
                                                      : BT::NodeStatus::FAILURE;
  }
};

}  // namespace coug_helm::bt_nodes
