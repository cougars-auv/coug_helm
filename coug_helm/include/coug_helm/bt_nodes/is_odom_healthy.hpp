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
 * @file is_odom_healthy.hpp
 * @brief BT condition node that checks odometry health.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <string>

namespace coug_helm::bt_nodes {

/**
 * @class IsOdomHealthy
 * @brief BT condition node that checks odometry health.
 */
class IsOdomHealthy : public BT::ConditionNode {
 public:
  IsOdomHealthy(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  // --- Overrides ---
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("last_odom_time"),
        BT::InputPort<double>("current_time"),
        BT::InputPort<double>("odom_timeout"),
    };
  }

  /**
   * @brief Checks whether odometry is being received within the timeout.
   * @return SUCCESS if odometry is healthy, FAILURE otherwise.
   */
  BT::NodeStatus tick() override {
    double last_odom = getInput<double>("last_odom_time").value();
    double now = getInput<double>("current_time").value();
    double timeout = getInput<double>("odom_timeout").value();

    if (last_odom == 0.0) {  // Treat "never received" as unhealthy.
      return BT::NodeStatus::FAILURE;
    }
    return ((now - last_odom) < timeout) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace coug_helm::bt_nodes
