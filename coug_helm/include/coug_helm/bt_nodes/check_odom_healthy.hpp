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
 * @file check_odom_healthy.hpp
 * @brief BT condition node that checks odometry health.
 * @author Nelson Durrant
 * @date May 2026
 */

#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <string>

namespace coug_helm::bt_nodes {

/**
 * @class CheckOdomHealthy
 * @brief Returns SUCCESS if odometry is arriving within the timeout window.
 */
class CheckOdomHealthy : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor for CheckOdomHealthy.
   * @param name The name of the node.
   * @param config The BT node configuration.
   */
  CheckOdomHealthy(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Checks whether odometry is being received within the timeout.
   * @return SUCCESS if odometry is healthy, FAILURE otherwise.
   */
  BT::NodeStatus tick() override {
    double last_odom = config().blackboard->get<double>("last_odom_time");
    double now = config().blackboard->get<double>("current_time");
    double timeout = config().blackboard->get<double>("odom_timeout_sec");

    if (last_odom == 0.0) {
      return BT::NodeStatus::FAILURE;
    }
    return ((now - last_odom) < timeout) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace coug_helm::bt_nodes
